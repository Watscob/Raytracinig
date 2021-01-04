#ifndef PROCEDURAL_BACKGROUND_H
#define PROCEDURAL_BACKGROUND_H

#include "image.h"
#include "scene.h"

void init_seed(int x);
void generate_noise_map(size_t width, size_t height, float scale);
void free_noise_map(void);
struct rgb_pixel get_procedural_pixel(struct scene *scene,
                                      struct rgb_image *image, size_t x,
                                      size_t y);
struct vec3 get_procedural_pixel_vec(struct scene *scene,
                                     struct rgb_image *image, size_t x,
                                     size_t y);

#endif /* PROCEDURAL_BACKGROUND_H */
