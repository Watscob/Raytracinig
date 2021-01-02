#include <err.h>
#include <limits.h>
#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "bmp.h"
#include "camera.h"
#include "image.h"
#include "normal_material.h"
#include "obj_loader.h"
#include "phong_material.h"
#include "procedural_background.h"
#include "scene.h"
#include "sphere.h"
#include "triangle.h"
#include "vec3.h"
#include "color.h"

#define NB_RAY_PER_PIXEL 5
#define NB_REC_REFLECTION 4
static double coor_offset[5][2] = {
    {0, 0},
    {-0.5, -0.5},
    {-0.5, 0.5},
    {0.5, -0.5},
    {0.5, 0.5},
};

static void build_test_scene(struct scene *scene, double aspect_ratio)
{
    // create a sample red material
    struct phong_material *red_material = zalloc(sizeof(*red_material));
    phong_material_init(red_material);
    red_material->surface_color = light_from_rgb_color(191, 32, 32);
    red_material->diffuse_Kn = 0.2;
    red_material->spec_n = 10;
    red_material->spec_Ks = 0.2;
    red_material->ambient_intensity = 0.1;

    // create a single sphere with the above material, and add it to the scene
    struct sphere *sample_sphere
        = sphere_create((struct vec3){0, 10, 0}, 4, &red_material->base);
    object_vect_push(&scene->objects, &sample_sphere->base);

    // go the same with a triangle
    // points are listed counter-clockwise
    //     a
    //    /|
    //   / |
    //  b--c
    struct vec3 points[3] = {
        {6, 10, 1}, // a
        {5, 10, 0}, // b
        {6, 10, 0}, // c
    };

    struct triangle *sample_triangle
        = triangle_create(points, &red_material->base);
    object_vect_push(&scene->objects, &sample_triangle->base);

    // setup the scene lighting
    scene->light_intensity = 5;
    scene->light_color = light_from_rgb_color(255, 255, 0); // yellow
    scene->light_direction = (struct vec3){-1, 1, -1};
    vec3_normalize(&scene->light_direction);

    // setup the camera
    double cam_width = 10;
    double cam_height = cam_width / aspect_ratio;

    scene->camera = (struct camera){
        .center = {0, 0, 0},
        .forward = {0, 1, 0},
        .up = {0, 0, 1},
        .width = cam_width,
        .height = cam_height,
        .focal_distance = focal_distance_from_fov(cam_width, 80),
    };

    // release the reference to the material
    material_put(&red_material->base);
}

static void build_obj_scene(struct scene *scene, double aspect_ratio)
{
    // setup the scene lighting
    scene->light_intensity = 5;
    scene->light_color = light_from_rgb_color(255, 255, 0); // yellow
    scene->light_direction = (struct vec3){-1, -1, -1};
    vec3_normalize(&scene->light_direction);

    // setup the camera
    double cam_width = 2;
    double cam_height = cam_width / aspect_ratio;

    // for some reason the object points in the z axis,
    // with its up on y
    scene->camera = (struct camera){
        .center = {0, 1, 2},
        .forward = {0, -1, -2},
        .up = {0, 1, 0},
        .width = cam_width,
        .height = cam_height,
        .focal_distance = focal_distance_from_fov(cam_width, 40),
    };

    vec3_normalize(&scene->camera.forward);
    vec3_normalize(&scene->camera.up);
}

static struct ray image_cast_ray(const struct rgb_image *image,
                                 const struct scene *scene, double x, double y)
{
    // find the position of the current pixel in the image plane
    // camera_cast_ray takes camera relative positions, from -0.5 to 0.5 for
    // both axis
    double cam_x = (x / image->width) - 0.5;
    double cam_y = (y / image->height) - 0.5;

    // find the starting point and direction of this ray
    struct ray ray;
    camera_cast_ray(&ray, &scene->camera, cam_x, cam_y);
    return ray;
}

static double
scene_intersect_ray(struct object_intersection *closest_intersection,
                    struct scene *scene, struct ray *ray)
{
    // we will now try to find the closest object in the scene
    // intersecting this ray
    double closest_intersection_dist = INFINITY;

    for (size_t i = 0; i < object_vect_size(&scene->objects); i++)
    {
        struct object *obj = object_vect_get(&scene->objects, i);
        struct object_intersection intersection;
        // if there's no intersection between the ray and this object, skip it
        double intersection_dist = obj->intersect(&intersection, obj, ray);
        if (intersection_dist >= closest_intersection_dist)
            continue;

        closest_intersection_dist = intersection_dist;
        *closest_intersection = intersection;
    }

    return closest_intersection_dist;
}

/* Handle reflection
*/
static void get_reflect_ray(struct scene *scene, struct ray *ray, struct object_intersection *closest_intersection)
{
    struct vec3 off = vec3_mul(&ray->direction, 0.01);
    ray->source = vec3_add(&closest_intersection->location.point, &off);
    ray->direction = vec3_reflect(&scene->light_direction, &closest_intersection->location.normal);
}

static struct vec3 reflect(struct rgb_image *image, struct scene *scene,
                           struct ray *ray, int rec, double x, double y)
{
    if (rec >= NB_REC_REFLECTION)
        return (struct vec3){0};

    // Get intersection
    struct object_intersection closest_intersection;
    double closest_intersection_dist
        = scene_intersect_ray(&closest_intersection, scene, ray);

    // If no intersection
    if (isinf(closest_intersection_dist))
        return get_procedural_pixel_vec(scene, image, x, y);

    // Get material
    struct material *mat = closest_intersection.material;
    struct vec3 pix_color = mat->shade(mat, &closest_intersection.location, scene, ray);

    // Create reflected ray
    get_reflect_ray(scene, ray, &closest_intersection);

    /* Add reflected ray to current color
    ** pixel_color += 0.2 * reflect()
    */
    struct vec3 ret_vec = reflect(image, scene, ray, rec + 1, x, y);
    ret_vec = vec3_mul(&ret_vec, 0.2);
    return vec3_add(&ret_vec, &pix_color);
}

typedef void (*render_mode_f)(struct rgb_image *, struct scene *, size_t x,
                              size_t y);

/* For all the pixels of the image, try to find the closest object
** intersecting the camera ray. If an object is found, shade the pixel to
** find its color.
*/
static void render_shaded(struct rgb_image *image, struct scene *scene,
                          size_t x, size_t y)
{
    struct ray ray;
    struct vec3 pix_color = {0};
    struct vec3 tmp;

    /* Throw NB_RAY_PER_PIXEL rays for one pixel (antialiasing)
    */
    for (short i = 0; i < NB_RAY_PER_PIXEL; i++)
    {
        ray = image_cast_ray(image, scene, x + coor_offset[i][0],
                                           y + coor_offset[i][1]);

        tmp = reflect(image, scene, &ray, 0, x + coor_offset[i][0],
                                             y + coor_offset[i][1]);

        tmp = vec3_div(&tmp, NB_RAY_PER_PIXEL);
        pix_color = vec3_add(&pix_color, &tmp);
    }

    rgb_image_set(image, x, y, rgb_color_from_light(&pix_color));
}

/* For all the pixels of the image, try to find the closest object
** intersecting the camera ray. If an object is found, shade the pixel to
** find its color.
*/
static void render_normals(struct rgb_image *image, struct scene *scene,
                           size_t x, size_t y)
{
    struct ray ray = image_cast_ray(image, scene, x, y);

    struct object_intersection closest_intersection;
    double closest_intersection_dist
        = scene_intersect_ray(&closest_intersection, scene, &ray);

    // if the intersection distance is infinite, do not shade the pixel
    if (isinf(closest_intersection_dist))
    {
        struct rgb_pixel pix = get_procedural_pixel(scene, image, x, y);
        rgb_image_set(image, x, y, pix);
        return;
    }

    struct material *mat = closest_intersection.material;
    struct vec3 pix_color = normal_material.shade(
        mat, &closest_intersection.location, scene, &ray);
    rgb_image_set(image, x, y, rgb_color_from_light(&pix_color));
}

/* For all the pixels of the image, try to find the closest object
** intersecting the camera ray. If an object is found, shade the pixel to
** find its color.
*/
static void render_distances(struct rgb_image *image, struct scene *scene,
                             size_t x, size_t y)
{
    struct ray ray = image_cast_ray(image, scene, x, y);

    struct object_intersection closest_intersection;
    double closest_intersection_dist
        = scene_intersect_ray(&closest_intersection, scene, &ray);

    // if the intersection distance is infinite, do not shade the pixel
    if (isinf(closest_intersection_dist))
    {
        struct rgb_pixel pix = get_procedural_pixel(scene, image, x, y);
        rgb_image_set(image, x, y, pix);
        return;
    }

    assert(closest_intersection_dist > 0);

    // distance from 0 to +inf
    // we want something from 0 to 1
    double depth_repr = 1 / (closest_intersection_dist + 1);
    uint8_t depth_intensity = depth_repr * 255;
    struct rgb_pixel pix_color
        = {depth_intensity, depth_intensity, depth_intensity};
    rgb_image_set(image, x, y, pix_color);
}

/* Worker for threads
*/
struct worker_args
{
    render_mode_f renderer;
    struct rgb_image *image;
    struct scene *scene;
    size_t min_y;
    size_t max_y;
};

static struct worker_args *init_worker_args(render_mode_f renderer,
                                            struct rgb_image *image,
                                            struct scene *scene,
                                            size_t line_per_process,
                                            size_t i)
{
    struct worker_args *wa = malloc(sizeof(struct worker_args));

    if (wa == NULL)
        err(1, "Not enough memory");

    wa->renderer = renderer;
    wa->image = image;
    wa->scene = scene;
    wa->min_y = i * line_per_process;
    if (image->height - (i + 1) * line_per_process < line_per_process)
        wa->max_y = image->height;
    else
        wa->max_y = (i + 1) * line_per_process;

    return wa;
}

static void *worker(void *args)
{
    struct worker_args* wa = args;

    for (size_t y = wa->min_y; y < wa->max_y; y++)
        for (size_t x = 0; x < wa->image->width; x++)
            wa->renderer(wa->image, wa->scene, x, y);

    free(args);
    pthread_exit(NULL);
}

/* Render the image by callong a thread each line_per_process
*/
static void handle_renderer(render_mode_f renderer,
                            struct rgb_image *image,
                            struct scene *scene)
{
    size_t nb_process = sysconf(_SC_NPROCESSORS_ONLN) / 2;
    size_t line_per_process = image->height / nb_process;
    pthread_t thrds[nb_process];

    for(size_t i = 0; i < nb_process; i++)
    {
        struct worker_args *wa = init_worker_args(renderer,
                                                  image,
                                                  scene,
                                                  line_per_process,
                                                  i);

        if (pthread_create(&thrds[i], NULL, worker, (void *)wa) != 0)
            err(1, "Fail to create thread");
    }

    for(size_t i = 0; i < nb_process; i++)
        if (pthread_join(thrds[i], NULL) != 0)
            err(1, "Fail to join thread");
}

int main(int argc, char *argv[])
{
    int rc;

    if (argc < 3)
        errx(1, "Usage: SCENE.obj OUTPUT.bmp [--normals] [--distances]");

    srand(time(NULL));
    struct scene scene;
    scene_init(&scene);

    // initialize the frame buffer (the buffer that will store the result of the
    // rendering)
    struct rgb_image *image = rgb_image_alloc(1000, 1000);

    // set all the pixels of the image to black
    struct rgb_pixel bg_color = {0};
    rgb_image_clear(image, &bg_color);

    // Init procedural background
    init_seed(50);
    generate_noise_map(image->width, image->height, 100);

    double aspect_ratio = (double)image->width / image->height;

    // build the scene
    build_obj_scene(&scene, aspect_ratio);
    //build_test_scene(&scene, aspect_ratio);

    if (load_obj(&scene, argv[1]))
        return 41;

    // parse options
    render_mode_f renderer = render_shaded;
    for (int i = 3; i < argc; i++)
    {
        if (strcmp(argv[i], "--normals") == 0)
            renderer = render_normals;
        else if (strcmp(argv[i], "--distances") == 0)
            renderer = render_distances;
    }

    // render all pixels
    handle_renderer(renderer, image, &scene);

    // write the rendered image to a bmp file
    FILE *fp = fopen(argv[2], "w");
    if (fp == NULL)
        err(1, "failed to open the output file");

    rc = bmp_write(image, ppm_from_ppi(80), fp);
    fclose(fp);

    // release resources
    scene_destroy(&scene);
    free_noise_map();
    free(image);
    return rc;
}
