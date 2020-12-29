#ifndef PROCEDURAL_BACKGROUND_H
#define PROCEDURAL_BACKGROUND_H

#include "image.h"


void init_seed(int x);
void generate_noise_map(size_t width, size_t height, float scale);
void free_noise_map(void);
struct rgb_pixel get_procedural_pixel(struct rgb_image *image, size_t x, size_t y);

#endif /* PROCEDURAL_BACKGROUND_H */
