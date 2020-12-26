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
#include "scene.h"
#include "sphere.h"
#include "triangle.h"
#include "vec3.h"

#define MIN(a, b) (a < b) ? a : b
#define MAX(a, b) (a > b) ? a : b

unsigned char perm[512] = {
    151,160,137,91,90,15,131,13,201,95,96,53,194,233,7,225,140,
    36,103,30,69,142,8,99,37,240,21,10,23,190,6,148,247,120,234,
    75,0,26,197,62,94,252,219,203,117,35,11,32,57,177,33,88,237,
    149,56,87,174,20,125,136,171,168,68,175,74,165,71,134,139,
    48,27,166,77,146,158,231,83,111,229,122,60,211,133,230,220,
    105,92,41,55,46,245,40,244,102,143,54,65,25,63,161,1,216,80,
    73,209,76,132,187,208,89,18,169,200,196,135,130,116,188,159,
    86,164,100,109,198,173,186,3,64,52,217,226,250,124,123,5,
    202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,
    182,189,28,42,223,183,170,213,119,248,152,2,44,154,163,70,
    221,153,101,155,167,43,172,9,129,22,39,253,19,98,108,110,79,
    113,224,232,178,185,112,104,218,246,97,228,251,34,242,193,
    238,210,144,12,191,179,162,241,81,51,145,235,249,14,239,107,
    49,192,214,31,181,199,106,157,184,84,204,176,115,121,50,45,
    127,4,150,254,138,236,205,93,222,114,67,29,24,72,243,141,
    128,195,78,66,215,61,156,180,
    151,160,137,91,90,15,131,13,201,95,96,53,194,233,7,225,140,
    127,4,150,254,138,236,205,93,222,114,67,29,24,72,243,141,
    128,195,78,66,215,61,156,180};

static int SEED = 0;
static float *noise_map = NULL;

static int noise2(size_t x, size_t y)
{
    int tmp = perm[(y + SEED) % 512];
    return perm[(tmp + x) % 512];
}

static float lin_inter(float x, float y, float s)
{
    return x + s * (y - x);
}

static float smooth_inter(float x, float y, float s)
{
    return lin_inter(x, y, s * s * (3-2*s));
}

static float noise2d(float x, float y)
{
    size_t x_int = x;
    size_t y_int = y;

    float x_frac = x - x_int;
    float y_frac = y - y_int;

    int s = noise2(x_int, y_int);
    int t = noise2(x_int + 1, y_int);
    int u = noise2(x_int, y_int + 1);
    int v = noise2(x_int + 1, y_int + 1);

    float low = smooth_inter(s, t, x_frac);
    float high = smooth_inter(u, v, x_frac);
    return smooth_inter(low, high, y_frac);
}

float perlin2d(float x, float y)
{
    float freq = 0.1;
    int depth = 4;

    float xa = x*freq;
    float ya = y*freq;
    float amp = 1.0;
    float fin = 0;
    float div = 0.0;

    for(int i = 0; i < depth; i++)
    {
        div += 256 * amp;
        fin += noise2d(xa, ya) * amp;
        amp /= 2;
        xa *= 2;
        ya *= 2;
    }

    return fin / div;
}

static float *generate_noise_map(size_t width, size_t height, float scale)
{
    if (scale <= 0)
        scale = 0.0001;

    float *noise_map = calloc(sizeof(float), height * width);
    if (noise_map == NULL)
        err(1, "Not enough memory");

    for (size_t y = 0; y < height; y++)
        for (size_t x = 0; x < width; x++)
            noise_map[y * width + x] = perlin2d(x / scale, y / scale);

    return noise_map;
}

static void free_noise_map(void)
{
    free(noise_map);
    noise_map = NULL;
}

/*
** The color of a light is encoded inside a float, from 0 to +inf,
** where 0 is no light, and +inf a lot more light. Unfortunately,
** regular images can't hold such a huge range, and each color channel
** is usualy limited to [0,255]. This function does the (lossy) translation
** by mapping the float [0,1] range to [0,255]
*/
static inline uint8_t translate_light_component(double light_comp)
{
    if (light_comp < 0.)
        light_comp = 0.;
    if (light_comp > 1.)
        light_comp = 1.;

    return light_comp * 255;
}

/*
** Converts an rgb floating point light color to 24 bit rgb.
*/
struct rgb_pixel rgb_color_from_light(const struct vec3 *light)
{
    struct rgb_pixel res;
    res.r = translate_light_component(light->x);
    res.g = translate_light_component(light->y);
    res.b = translate_light_component(light->z);
    return res;
}

static void build_test_scene(struct scene *scene, double aspect_ratio)
{
    // create a sample red material
    struct phong_material *red_material = zalloc(sizeof(*red_material));
    phong_material_init(red_material);
    red_material->surface_color = (struct vec3){0.75, 0.125, 0.125};
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
    scene->light_color = (struct vec3){1, 0.2, 0.2};
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
    scene->light_color = (struct vec3){1, 0.2, 0.2};
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
                                 const struct scene *scene, size_t x, size_t y)
{
    // find the position of the current pixel in the image plane
    // camera_cast_ray takes camera relative positions, from -0.5 to 0.5 for
    // both axis
    double cam_x = ((double)x / image->width) - 0.5;
    double cam_y = ((double)y / image->height) - 0.5;

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

typedef void (*render_mode_f)(struct rgb_image *, struct scene *, size_t x,
                              size_t y);

/* For all the pixels of the image, try to find the closest object
** intersecting the camera ray. If an object is found, shade the pixel to
** find its color.
*/
static void render_shaded(struct rgb_image *image, struct scene *scene,
                          size_t x, size_t y)
{
    struct ray ray = image_cast_ray(image, scene, x, y);

    struct object_intersection closest_intersection;
    double closest_intersection_dist
        = scene_intersect_ray(&closest_intersection, scene, &ray);

    // if the intersection distance is infinite, do not shade the pixel
    if (isinf(closest_intersection_dist))
        return;

    struct material *mat = closest_intersection.material;
    struct vec3 pix_color
        = mat->shade(mat, &closest_intersection.location, scene, &ray);

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
        return;

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
        return;

    assert(closest_intersection_dist > 0);

    double depth_repr = 1 / (closest_intersection_dist + 1);
    uint8_t depth_intensity = translate_light_component(depth_repr);
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
    wa->max_y = (i + 1) * line_per_process + 1;

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

/* Antialiasing function (take the medium of  4 pixels to do 1 pixel
*/
struct rgb_image *reduce_image(struct rgb_image *image)
{
    struct rgb_image *res = rgb_image_alloc(image->width / 2,
                                            image->height / 2);

    for(size_t y = 0; y < res->height; y++)
    {
        for(size_t x = 0; x < res->width; x++)
        {
            res->data[res->width * y + x].r =
                (image->data[image->width * 2 * y + 2 * x].r
                + image->data[image->width * (2 * y + 1) + 2 * x].r
                + image->data[image->width * 2 * y + (2 * x + 1)].r
                + image->data[image->width * (2 * y + 1) + (2 * x + 1)].r) / 4;
            res->data[res->width * y + x].g =
                (image->data[image->width * 2 * y + 2 * x].g
                + image->data[image->width * (2 * y + 1) + 2 * x].g
                + image->data[image->width * 2 * y + (2 * x + 1)].g
                + image->data[image->width * (2 * y + 1) + (2 * x + 1)].g) / 4;
            res->data[res->width * y + x].b =
                (image->data[image->width * 2 * y + 2 * x].b
                + image->data[image->width * (2 * y + 1) + 2 * x].b
                + image->data[image->width * 2 * y + (2 * x + 1)].b
                + image->data[image->width * (2 * y + 1) + (2 * x + 1)].b) / 4;
        }
    }

    free(image);
    return res;
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
    struct rgb_image *image = rgb_image_alloc(2000, 2000);

    SEED = rand() % (10);
    noise_map = generate_noise_map(image->width, image->height, 25);

    /*for(size_t y = 0; y < image->height; y++)
    {
        for(size_t x = 0; x < image->width; x++)
        {
            float noise = noise_map[y * image->width + x];
            struct rgb_pixel pix =
            {
                .r = 255 * noise,
                .g = 255 * noise,
                .b = 255 * noise,
            };
            rgb_image_set(image, x, y, pix);
        }
    }*/
    struct rgb_pixel sky = {.r = 135, .g = 206, .b = 235};
    struct rgb_pixel snow = {.r = 255, .g = 255, .b = 255};
    struct rgb_pixel ground = {.r = 155, .g = 118, .b = 83};
    struct rgb_pixel grass = {.r = 86, .g = 125, .b = 70};

    // set all the pixels of the image to black
    struct rgb_pixel bg_color = sky;
    rgb_image_clear(image, &bg_color);

    int r = rand() % image->height;

    float ratio = 1.5;

    size_t *save_last_y_max = malloc(sizeof(size_t) * image->width);
    for (size_t i = 0; i < image->width; i++)
        save_last_y_max[i] = UINT_MAX;

    for (size_t i = 0; i < 3; i++)
    {
        for (size_t x = 0; x < image->width; x++)
        {
            float noise = noise_map[r * image->width + x];

            size_t y_max = MIN(noise * image->height / ratio, save_last_y_max[x]);
            if (i == 0)
                save_last_y_max[x] = y_max;

            for(size_t y = 0; y < y_max; y++)
            {
                struct rgb_pixel pix;

                if (i == 0)
                    pix = snow;
                else if (i == 1)
                    pix = ground;
                else if (i == 2)
                    pix = grass;

                rgb_image_set(image, x, y, pix);
            }
        }
        r = rand() % image->height;
        if (i == 0)
            ratio *= 1.7;
        else if (i == 1)
            ratio *= 1.2;
    }

    free_noise_map();

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

    // apply anti-aliasing
    image = reduce_image(image);


    // write the rendered image to a bmp file
    FILE *fp = fopen(argv[2], "w");
    if (fp == NULL)
        err(1, "failed to open the output file");

    rc = bmp_write(image, ppm_from_ppi(80), fp);
    fclose(fp);

    // release resources
    scene_destroy(&scene);
    free(image);
    return rc;
}
