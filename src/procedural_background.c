#include "procedural_background.h"

#include <err.h>
#include <stdlib.h>
#include <time.h>

static float *noise_map = NULL;

unsigned char perm[512] = {
    151, 160, 137, 91,  90,  15,  131, 13,  201, 95,  96,  53,  194, 233, 7,
    225, 140, 36,  103, 30,  69,  142, 8,   99,  37,  240, 21,  10,  23,  190,
    6,   148, 247, 120, 234, 75,  0,   26,  197, 62,  94,  252, 219, 203, 117,
    35,  11,  32,  57,  177, 33,  88,  237, 149, 56,  87,  174, 20,  125, 136,
    171, 168, 68,  175, 74,  165, 71,  134, 139, 48,  27,  166, 77,  146, 158,
    231, 83,  111, 229, 122, 60,  211, 133, 230, 220, 105, 92,  41,  55,  46,
    245, 40,  244, 102, 143, 54,  65,  25,  63,  161, 1,   216, 80,  73,  209,
    76,  132, 187, 208, 89,  18,  169, 200, 196, 135, 130, 116, 188, 159, 86,
    164, 100, 109, 198, 173, 186, 3,   64,  52,  217, 226, 250, 124, 123, 5,
    202, 38,  147, 118, 126, 255, 82,  85,  212, 207, 206, 59,  227, 47,  16,
    58,  17,  182, 189, 28,  42,  223, 183, 170, 213, 119, 248, 152, 2,   44,
    154, 163, 70,  221, 153, 101, 155, 167, 43,  172, 9,   129, 22,  39,  253,
    19,  98,  108, 110, 79,  113, 224, 232, 178, 185, 112, 104, 218, 246, 97,
    228, 251, 34,  242, 193, 238, 210, 144, 12,  191, 179, 162, 241, 81,  51,
    145, 235, 249, 14,  239, 107, 49,  192, 214, 31,  181, 199, 106, 157, 184,
    84,  204, 176, 115, 121, 50,  45,  127, 4,   150, 254, 138, 236, 205, 93,
    222, 114, 67,  29,  24,  72,  243, 141, 128, 195, 78,  66,  215, 61,  156,
    180, 151, 160, 137, 91,  90,  15,  131, 13,  201, 95,  96,  53,  194, 233,
    7,   225, 140, 127, 4,   150, 254, 138, 236, 205, 93,  222, 114, 67,  29,
    24,  72,  243, 141, 128, 195, 78,  66,  215, 61,  156, 180};

static int SEED = 0;

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
    return lin_inter(x, y, s * s * (3 - 2 * s));
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

static float perlin2d(float x, float y)
{
    float freq = 0.1;
    int depth = 4;

    float xa = x * freq;
    float ya = y * freq;
    float amp = 1.0;
    float fin = 0;
    float div = 0.0;

    for (int i = 0; i < depth; i++)
    {
        div += 256 * amp;
        fin += noise2d(xa, ya) * amp;
        amp /= 2;
        xa *= 2;
        ya *= 2;
    }

    return fin / div;
}

void init_seed(int x)
{
    srand(time(NULL));
    SEED = rand() % x;
}

void generate_noise_map(size_t width, size_t height, float scale)
{
    if (scale <= 0)
        scale = 0.0001;

    noise_map = calloc(sizeof(float), height * width);
    if (noise_map == NULL)
        err(1, "Not enough memory");

    for (size_t y = 0; y < height; y++)
        for (size_t x = 0; x < width; x++)
            noise_map[y * width + x] = perlin2d(x / scale, y / scale);
}

void free_noise_map(void)
{
    free(noise_map);
    noise_map = NULL;
}

struct rgb_pixel get_procedural_pixel(struct scene *scene,
                                      struct rgb_image *image, size_t x,
                                      size_t y)
{
    float noise = noise_map[y * image->width + x];
    struct rgb_pixel pix = {.r = scene->light_color.x * 255 * noise * 0.05,
                            .g = scene->light_color.y * 255 * noise * 0.05,
                            .b = scene->light_color.z * 255 * noise * 0.05};

    return pix;
}

struct vec3 get_procedural_pixel_vec(struct scene *scene,
                                      struct rgb_image *image, size_t x,
                                      size_t y)
{
    float noise = noise_map[y * image->width + x];
    struct vec3 pix = {.x = scene->light_color.x * noise * 0.05,
                       .y = scene->light_color.y * noise * 0.05,
                       .z = scene->light_color.z * noise * 0.05};
    return pix;
}
