#ifndef KDTREE_H
#define KDTREE_H

#include <stdbool.h>

#include "object.h"
#include "ray.h"
#include "scene.h"
#include "vec3.h"

#define MIN_2(a, b) (a < b) ? a : b
#define MAX_2(a, b) (a > b) ? a : b

#define MIN_3(a, b, c) (a < b) ? ((a < c) ? a : c) : ((b < c) ? b : c)
#define MAX_3(a, b, c) (a > b) ? ((a > c) ? a : c) : ((b > c) ? b : c)

struct kdtree
{
    struct kdtree *left;
    struct kdtree *right;

    enum type
    {
        LEAF,
        NODE,
    } type;

    size_t size_list;
    union
    {
        struct kdtree **box_list;
        struct object *obj;
    } data;

    struct vec3 corner1;
    struct vec3 corner2;
};

struct kdtree *build_kdtree(struct scene *scene);
void free_kdtree(struct kdtree *root);
double kdtree_scene_intersect_ray(struct object_intersection *closest_intersection,
                                         struct kdtree *tree,
                                         struct ray *ray);

#endif /* KDTREE_H */
