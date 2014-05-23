#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ros/package.h>

int multiplier;
const int distance_between_wheels = 72;

int main() {
    int number_of_seeds, number_of_intermediate_points, layers;
    double velocity_ratio, final_x, final_y, final_orientation;
    FILE *seed_output;
    std::string seeds_file;
    seeds_file = ros::package::getPath("local_planner") + "/seeds/seeds8.txt";
    seed_output = fopen(seeds_file.c_str(), "w");

    //printf("OutputFormat:\nnumberOfSseeds\nvelocityRatio finalX finalY finalOrientation\nSeedPoints\nintermediateXValue intermediateYValue\n");

    printf("Enter number of seeds to be generated per quadrant\n");
    scanf("%d", &number_of_seeds);

    printf("Enter number of layers\n");
    scanf("%d", &layers);

    fprintf(seed_output, "%d\n", (4 * number_of_seeds + 2) * layers);

    for (int x = 1; x <= layers; x++) {
        multiplier = (x * 150 / number_of_seeds) / layers;
        printf("%d\n", multiplier);

        for (int i = 0; i < number_of_seeds; ++i) {

            final_x = (i + 1) * multiplier;
            final_y = (2 * number_of_seeds - i) * multiplier;

            double radius_of_curvature = (final_x * final_x + final_y * final_y) / (2 * final_x);
            double theta = asinf(final_y / radius_of_curvature);

            final_orientation = M_PI / 2 - theta;
            velocity_ratio = (radius_of_curvature + distance_between_wheels / 2) / (radius_of_curvature - distance_between_wheels / 2);

            fprintf(seed_output, "%lf %lf %lf %lf\n", velocity_ratio, final_x, final_y, final_orientation * 180 / M_PI);

            number_of_intermediate_points = (final_x + final_y) / 10;
            fprintf(seed_output, "%d\n", number_of_intermediate_points);

            double tempX = 0, tempY = 0, temp_orientation = theta / (2 * number_of_intermediate_points);
            double l = (2 * radius_of_curvature * sinf(theta / (2 * number_of_intermediate_points)));

            for (int j = 0; j < number_of_intermediate_points; ++j) {
                tempX += l * sinf(temp_orientation);
                tempY += l * cosf(temp_orientation);
                temp_orientation += theta / number_of_intermediate_points;

                fprintf(seed_output, "%lf %lf\n", tempX, tempY);
            }
        }

        for (int i = 0; i < number_of_seeds; ++i) {

            final_x = (i + 1) * multiplier;
            final_y = (2 * number_of_seeds - i) * multiplier;

            double radius_of_curvature = (final_x * final_x + final_y * final_y) / (2 * final_x);
            double theta = asinf(final_y / radius_of_curvature);

            final_orientation = M_PI / 2 - theta;
            velocity_ratio = 1 - distance_between_wheels / radius_of_curvature;

            fprintf(seed_output, "%lf %lf %lf %lf\n", velocity_ratio, -final_x, final_y, 180 - final_orientation * 180 / M_PI);

            number_of_intermediate_points = (final_x + final_y) / 10;
            fprintf(seed_output, "%d\n", number_of_intermediate_points);

            double tempX = 0, tempY = 0, temp_orientation = theta / (2 * number_of_intermediate_points);
            double l = (2 * radius_of_curvature * sinf(theta / (2 * number_of_intermediate_points)));

            for (int j = 0; j < number_of_intermediate_points; ++j) {
                tempX += l * sinf(temp_orientation);
                tempY += l * cosf(temp_orientation);
                temp_orientation += theta / number_of_intermediate_points;
                fprintf(seed_output, "%lf %lf\n", -tempX, tempY);
            }
        }

        //generating straight seed
        velocity_ratio = 1.0;
        final_x = 0;
        final_y = (2 * number_of_seeds) * multiplier;
        final_orientation = M_PI / 2;

        fprintf(seed_output, "%lf %lf %lf %lf\n", velocity_ratio, final_x, final_y, final_orientation * 180 / M_PI);

        number_of_intermediate_points = (final_x + final_y) / 10;
        fprintf(seed_output, "%d\n", number_of_intermediate_points);

        for (int i = 1; i <= number_of_intermediate_points; ++i) {
            fprintf(seed_output, "%lf %lf\n", final_x, final_y * i / number_of_intermediate_points);
        }

        for (int i = 0; i < number_of_seeds; ++i) {
            final_x = (i + 1) * multiplier;
            final_y = (2 * number_of_seeds - i) * multiplier;

            double radius_of_curvature = (final_x * final_x + final_y * final_y) / (2 * final_x);
            double theta = asinf(final_y / radius_of_curvature);

            final_orientation = M_PI / 2 - theta;
            velocity_ratio = 1 + distance_between_wheels / radius_of_curvature;
            fprintf(seed_output, "%lf %lf %lf %lf\n", velocity_ratio, -final_x, -final_y, 180 + final_orientation * 180 / M_PI);

            number_of_intermediate_points = (final_x + final_y) / 10;
            fprintf(seed_output, "%d\n", number_of_intermediate_points);

            double temp_x = 0, temp_y = 0, temp_orientation = theta / (2 * number_of_intermediate_points);

            double length = (2 * radius_of_curvature * sinf(theta / (2 * number_of_intermediate_points)));

            for (int j = 0; j < number_of_intermediate_points; ++j) {
                temp_x += length * sinf(temp_orientation);
                temp_y += length * cosf(temp_orientation);
                temp_orientation += theta / number_of_intermediate_points;
                fprintf(seed_output, "%lf %lf\n", -temp_x, -temp_y);
            }
        }

        for (int i = 0; i < number_of_seeds; ++i) {
            final_x = (i + 1) * multiplier;
            final_y = (2 * number_of_seeds - i) * multiplier;

            double radius_of_curvature = (final_x * final_x + final_y * final_y) / (2 * final_x);
            double theta = asinf(final_y / radius_of_curvature);

            final_orientation = M_PI / 2 - theta;
            velocity_ratio = 1 + distance_between_wheels / radius_of_curvature;

            fprintf(seed_output, "%lf %lf %lf %lf\n", velocity_ratio, final_x, -final_y, 360 - final_orientation * 180 / M_PI);

            number_of_intermediate_points = (final_x + final_y) / 10;
            fprintf(seed_output, "%d\n", number_of_intermediate_points);

            double temp_x = 0, temp_y = 0, temp_orientation = theta / (2 * number_of_intermediate_points);
            double l = (2 * radius_of_curvature * sinf(theta / (2 * number_of_intermediate_points)));

            for (int j = 0; j < number_of_intermediate_points; ++j) {
                temp_x += l * sinf(temp_orientation);
                temp_y += l * cosf(temp_orientation);
                temp_orientation += theta / number_of_intermediate_points;
                fprintf(seed_output, "%lf %lf\n", temp_x, -temp_y);
            }
        }

        //generating straight seed
        velocity_ratio = 1.0;
        final_x = 0;
        final_y = (2 * number_of_seeds) * multiplier;
        final_orientation = M_PI / 2;
        fprintf(seed_output, "%lf %lf %lf %lf\n", velocity_ratio, final_x, -final_y, final_orientation * 180 / M_PI);

        number_of_intermediate_points = (final_x + final_y) / 10;
        fprintf(seed_output, "%d\n", number_of_intermediate_points);

        for (int i = 1; i <= number_of_intermediate_points; ++i) {
            fprintf(seed_output, "%lf %lf\n", final_x, -final_y * i / number_of_intermediate_points);
        }
    }
    return 0;
}

/*
Documentation:
Input: Number of seeds per quadrant
So, total number of seeds = 2*numberOfSeeds + 1 (1 extra for the straight seed)
For I quadrant,
Starting point = (0,0)
The end point (finalX, finalY) of seeds are assumed to lie on a straight line starting from 2*numberOfSeeds and inclined at -45degrees.
The final orientation is the angle the tangent at the end point makes with x axis.
For each end point, a circular arc is plotted with the following constraints:
1. End points must be a whole number (lattice point)
2. Tangent at start point must be along y axis
3. The final orientation must not be negative
The centre of circle lies on +x axis, its radius = x-coordinate of centre.
The angle which the arc subtends at centre is Theta
The final orientation is calc from Theta
Cost of seeds is approximated so that it increases gradually as the curvature increases
Cost of straight seed is kept minimum.
Theta is divided into numberOfSeeds equal parts.
Intermediate points of the curve is obtained by adding components to tempX and tempY, the lenth of straight line between two intemediate points denoted by 'l'

For II quadrant
tempX and finalOrientation is complemented.

Lastly, the straight seed is generated.
 */