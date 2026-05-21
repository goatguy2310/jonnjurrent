#pragma once

#include "../geometry/scene.h"

#include "../stb/stb_image_write.h"
#include "../stb/stb_image.h"

class Renderer {
public:
	explicit Renderer() {}

	inline void render(std::string filename, Scene& scene) {
		std::vector<unsigned char> image(W * H * 3, 0);

		double z = -W / (2. * std::tan(scene.fov / 2.));
#pragma omp parallel for schedule(static)
		for (int i = 0; i < H; i++) {
			for (int j = 0; j < W; j++) {
				Vector color(0., 0., 0.);

				double x = j - W / 2. + 0.5;
				double y = H / 2. - i + 0.5;

				double stddev = 0.4;
				for (int it = 0; it < sample_count; it++) {
					double r1 = getRandom();
					double r2 = getRandom();

					double sx = stddev * std::sqrt(-2 * std::log(r1)) * std::cos(2 * M_PI * r2);
					double sy = stddev * std::sqrt(-2 * std::log(r1)) * std::sin(2 * M_PI * r2);

					Vector ray_direction(x + sx, y + sy, z);
					ray_direction.normalize();

					Ray ray(scene.camera_center, ray_direction);
					color = color + scene.getColor(ray, 0);
				}
				color = color / sample_count;

				image[(i * W + j) * 3 + 0] = std::min(255., std::max(0., 255. * std::pow(color[0] / 255., 1. / scene.gamma)));
				image[(i * W + j) * 3 + 1] = std::min(255., std::max(0., 255. * std::pow(color[1] / 255., 1. / scene.gamma)));
				image[(i * W + j) * 3 + 2] = std::min(255., std::max(0., 255. * std::pow(color[2] / 255., 1. / scene.gamma)));
			}
		}
		stbi_write_png(filename.c_str(), W, H, 3, &image[0], 0);
	}

	int W, H, sample_count;
};
