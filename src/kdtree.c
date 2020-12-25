#include <err.h>
#include <stdbool.h>
#include <stdlib.h>

#include "kdtree.h"
#include "scene.h"
#include "triangle.h"
#include "vec3.h"

static struct kdtree *init_kdtree_leaf(struct object *obj,
                                       struct vec3 corner1,
                                       struct vec3 corner2)
{
    struct kdtree *tmp = calloc(sizeof(struct kdtree), 1);

    if (tmp == NULL)
        err(1, "Not enough memory");

    tmp->type = LEAF;
    tmp->data.obj = obj;
    tmp->corner1 = corner1;
    tmp->corner2 = corner2;

    return tmp;
}

static struct kdtree *init_kdtree_node(struct kdtree **box_list,
                                       struct vec3 corner1,
                                       struct vec3 corner2)
{
    struct kdtree *tmp = calloc(sizeof(struct kdtree), 1);

    if (tmp == NULL)
        err(1, "Not enough memory");

    tmp->type = NODE;
    tmp->data.box_list = box_list;
    tmp->corner1 = corner1;
    tmp->corner2 = corner2;

    return tmp;
}

static struct kdtree **get_list_box(struct scene *scene, size_t size_objs)
{
    struct kdtree **box_list = malloc(sizeof(struct kdtree *) * size_objs);

    for (size_t i = 0; i < size_objs; i++)
    {
        struct triangle *trian = (struct triangle *)object_vect_get(&scene->objects, i);
        struct vec3 corner1 =
        {
            .x = MIN_3(trian->points[0].x, trian->points[1].x, trian->points[2].x),
            .y = MIN_3(trian->points[0].y, trian->points[1].y, trian->points[2].y),
            .z = MIN_3(trian->points[0].z, trian->points[1].z, trian->points[2].z),
        };
        struct vec3 corner2 =
        {
            .x = MAX_3(trian->points[0].x, trian->points[1].x, trian->points[2].x),
            .y = MAX_3(trian->points[0].y, trian->points[1].y, trian->points[2].y),
            .z = MAX_3(trian->points[0].z, trian->points[1].z, trian->points[2].z),
        };

        box_list[i] = init_kdtree_leaf((struct object *)trian, corner1, corner2);
    }
    return box_list;
}

static struct kdtree *merge_box(struct kdtree *box1, struct kdtree *box2)
{
    if (box1 == NULL)
        return init_kdtree_node(NULL, box2->corner1, box2->corner2);

    if (box2 == NULL)
        return init_kdtree_node(NULL, box1->corner1, box1->corner2);

    struct vec3 corner1 =
    {
        .x = MIN_2(box1->corner1.x, box2->corner1.x),
        .y = MIN_2(box1->corner1.y, box2->corner1.y),
        .z = MIN_2(box1->corner1.z, box2->corner1.z),
    };
    struct vec3 corner2 =
    {
        .x = MAX_2(box1->corner2.x, box2->corner2.x),
        .y = MAX_2(box1->corner2.y, box2->corner2.y),
        .z = MAX_2(box1->corner2.z, box2->corner2.z),
    };

    return init_kdtree_node(NULL, corner1, corner2);
}

static bool intersect_box(struct kdtree *box1, struct kdtree *box2)
{
    return ((box1->corner1.x < box2->corner2.x && box1->corner2.x > box2->corner1.x)
        && (box1->corner1.y < box2->corner2.y && box1->corner2.y > box2->corner1.y)
        && (box1->corner1.z < box2->corner2.z && box1->corner2.z > box2->corner1.z));
/*        || ((box2->corner1.x < box1->corner2.x && box2->corner2.x > box1->corner1.x)
        && (box2->corner1.y < box1->corner2.y && box2->corner2.y > box1->corner1.y)
        && (box2->corner1.z < box1->corner2.z && box2->corner2.z > box1->corner1.z));*/
}

static void get_childrens(struct kdtree *root, struct kdtree **left, struct kdtree **right, char c)
{
    if (c == 'x')
    {
        *left = init_kdtree_node(NULL,
                root->corner1,
                (struct vec3)
                {.x = MAX_2(root->corner1.x, root->corner2.x) / 2,
                .y = root->corner2.y,
                .z = root->corner2.z});
        *right = init_kdtree_node(NULL,
                (struct vec3)
                {.x = MAX_2(root->corner1.x, root->corner2.x) / 2,
                .y = root->corner1.y,
                .z = root->corner1.z},
                root->corner2);
    }
    else if (c == 'y')
    {
        *left = init_kdtree_node(NULL,
                root->corner1,
                (struct vec3)
                {.x = root->corner2.x,
                .y = MAX_2(root->corner1.z, root->corner2.y) / 2,
                .z = root->corner2.z});
        *right = init_kdtree_node(NULL,
                (struct vec3)
                {.x = root->corner1.x,
                .y = MAX_2(root->corner1.y, root->corner2.y) / 2,
                .z = root->corner1.z},
                root->corner2);
    }
    else
    {
        *left = init_kdtree_node(NULL,
                root->corner1,
                (struct vec3)
                {.x = root->corner2.x,
                .y = root->corner2.y,
                .z = MAX_2(root->corner1.z, root->corner2.z) / 2});
        *right = init_kdtree_node(NULL,
                (struct vec3)
                {.x = root->corner1.x,
                .y = root->corner1.y,
                .z = MAX_2(root->corner1.z, root->corner2.z) / 2},
                root->corner2);
    }
}

static void build_kdtree_rec(struct kdtree *root, size_t nb)
{
    if (nb == 25)
        return;

    struct kdtree *left = NULL;
    struct kdtree *right = NULL;

    char coor = 'x';

    if (nb % 3 == 1)
        coor = 'y';
    else if (nb % 3 == 2)
        coor = 'z';

    get_childrens(root, &left, &right, coor);

    struct kdtree **box_list_left = calloc(sizeof(struct kdtree *), root->size_list);
    size_t index_left = 0;
    struct kdtree **box_list_right = calloc(sizeof(struct kdtree *), root->size_list);
    size_t index_right = 0;

    for (size_t i = 0; i < root->size_list; i++)
    {
        if (intersect_box(root->data.box_list[i], left))
        {
            box_list_left[index_left] = root->data.box_list[i];
            index_left++;
        }

        if (intersect_box(root->data.box_list[i], right))
        {
            box_list_right[index_right] = root->data.box_list[i];
            index_right++;
        }
    }

    if (index_left == 0)
    {
        free(left);
        free(box_list_left);
    }
    else
    {
        box_list_left = realloc(box_list_left, sizeof(struct kdtree *) * index_left);
        left->data.box_list = box_list_left;
        left->size_list = index_left;
        root->left = left;
        build_kdtree_rec(left, nb + 1);
    }

    if (index_right == 0)
    {
        free(right);
        free(box_list_right);
    }
    else
    {
        box_list_right = realloc(box_list_right, sizeof(struct kdtree *) * index_right);
        right->data.box_list = box_list_right;
        right->size_list = index_right;
        root->right = right;
        build_kdtree_rec(right, nb + 1);
    }
}

struct kdtree *build_kdtree(struct scene *scene)
{
    size_t size_objs = object_vect_size(&scene->objects);

    struct kdtree **box_list = get_list_box(scene, size_objs);

    struct kdtree *root = NULL;

    for (size_t i = 0; i < size_objs; i++)
    {
        struct kdtree *tmp = merge_box(root, box_list[i]);
        free(root);
        root = tmp;
    }

    root->size_list = size_objs;
    root->data.box_list = box_list;

    build_kdtree_rec(root, 0);

    return root;
}

void free_kdtree(struct kdtree *root)
{
    if (root == NULL)
        return;

    free_kdtree(root->left);
    free_kdtree(root->right);

    free(root->data.box_list);

    free(root);
}

static void swap_double(double *a, double *b)
{
    double tmp = *a;
    *a = *b;
    *b = tmp;
}

static bool intersect_box_ray(struct kdtree *box, struct ray *ray)
{
    double tmin = (box->corner1.x - ray->source.x) / ray->direction.x;
    double tmax = (box->corner2.x - ray->source.x) / ray->direction.x;

    if (tmin > tmax)
        swap_double(&tmin, &tmax);

    double tymin = (box->corner1.y - ray->source.y) / ray->direction.y;
    double tymax = (box->corner2.y - ray->source.y) / ray->direction.y;

    if (tymin > tymax)
        swap_double(&tymin, &tymax);

    if (tmin > tymax || tymin > tmax)
        return false;

    tmin = MAX_2(tmin, tymin);
    tmax = MIN_2(tmax, tymax);

    double tzmin = (box->corner1.z - ray->source.z) / ray->direction.z;
    double tzmax = (box->corner2.z - ray->source.z) / ray->direction.z;

    if (tzmin > tzmax)
        swap_double(&tzmin, &tzmax);

    return !(tmin > tzmax || tzmin > tmax);
}

static double get_object_intersect(struct kdtree *tree,
                                   struct ray *ray,
                                   struct object_intersection *closest_intersection)
{
    double closest_intersection_dist = INFINITY;
    if (tree->left != NULL && intersect_box_ray(tree->left, ray))
        closest_intersection_dist =
            MIN_2(closest_intersection_dist,
                  get_object_intersect(tree->left, ray, closest_intersection));

    if (tree->right != NULL && intersect_box_ray(tree->right, ray))
        closest_intersection_dist =
            MIN_2(closest_intersection_dist,
                  get_object_intersect(tree->right, ray, closest_intersection));

    if (tree->left != NULL || tree->right != NULL)
        return closest_intersection_dist;

    for (size_t i = 0; i < tree->size_list; i++)
    {
        struct object *obj = tree->data.box_list[i]->data.obj;
        struct object_intersection intersection;

        double intersection_dist = obj->intersect(&intersection, obj, ray);
        if (intersection_dist >= closest_intersection_dist)
            continue;

        closest_intersection_dist = intersection_dist;
        *closest_intersection = intersection;
    }

    return closest_intersection_dist;
}

double kdtree_scene_intersect_ray(struct object_intersection *closest_intersection,
                                         struct kdtree *tree,
                                         struct ray *ray)
{
    if (!intersect_box_ray(tree, ray))
        return INFINITY;

    return get_object_intersect(tree, ray, closest_intersection);
}
