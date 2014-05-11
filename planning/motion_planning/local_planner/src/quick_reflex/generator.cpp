#include <stdio.h>
#include <cmath>
#include <algorithm>

#include "ros/package.h"

int MULTIPLIER;
const int distanceBetweenWheels = 17;

int main()
{
	int numberOfSeeds, numberOfIntermediatePoints, layers;
	double velocityRatio, finalX, finalY, finalOrientation;

	FILE *seedOutput;
	
	std::string SEEDS_FILE;
	SEEDS_FILE = ros::package::getPath("backup_planner")+"/seeds/seeds.txt";
	
	seedOutput=fopen(SEEDS_FILE.c_str(),"w");
	
	//printf("OutputFormat:\nnumberOfSseeds\nvelocityRatio finalX finalY finalOrientation\nSeedPoints\nintermediateXValue intermediateYValue\n");
	
	printf("Enter number of seeds to be generated per quadrant\n");
	scanf("%d", &numberOfSeeds);

	printf("Enter number of layers\n");
	scanf("%d", &layers);


	fprintf(seedOutput, "%d\n", (4*numberOfSeeds+2)*layers);

	for(int x = 1; x <= layers; x++){

		MULTIPLIER = (x * 150 / numberOfSeeds) / layers;
		printf("%d\n",MULTIPLIER);


		for(int i = 0; i < numberOfSeeds; ++i){
		
			finalX = (i + 1)*MULTIPLIER;
			finalY = (2*numberOfSeeds-i)*MULTIPLIER;

			double radiusOfCurvature = (finalX*finalX + finalY*finalY)/(2 * finalX);
			double theta = asinf (finalY / radiusOfCurvature);

			finalOrientation = M_PI/2 - theta;
			velocityRatio = 1 + distanceBetweenWheels / radiusOfCurvature;

			fprintf(seedOutput, "%lf %lf %lf %lf\n", velocityRatio, finalX, finalY, finalOrientation*180/M_PI);

			numberOfIntermediatePoints = (finalX + finalY)/10;
			fprintf(seedOutput, "%d\n", numberOfIntermediatePoints);

			double tempX=0, tempY=0, tempOrientation=theta/(2*numberOfIntermediatePoints);

			double l = (2 * radiusOfCurvature *sinf(theta/(2*numberOfIntermediatePoints)));

			for(int j = 0; j < numberOfIntermediatePoints; ++j){
				tempX += l*sinf(tempOrientation);
				tempY += l*cosf(tempOrientation);
				tempOrientation += theta/numberOfIntermediatePoints;

				fprintf(seedOutput,"%lf %lf\n", tempX, tempY);
			}

		}


		for(int i = 0; i < numberOfSeeds; ++i){
		
			finalX = (i + 1)*MULTIPLIER;
			finalY = (2*numberOfSeeds-i)*MULTIPLIER;

			double radiusOfCurvature = (finalX*finalX + finalY*finalY)/(2 * finalX);
			double theta = asinf (finalY / radiusOfCurvature);

			finalOrientation = M_PI/2 - theta;
			velocityRatio = 1 - distanceBetweenWheels / radiusOfCurvature;;

			fprintf(seedOutput, "%lf %lf %lf %lf\n", velocityRatio, -finalX, finalY, 180 - finalOrientation*180/M_PI);

			numberOfIntermediatePoints = (finalX + finalY)/10;
			fprintf(seedOutput, "%d\n", numberOfIntermediatePoints);

			double tempX=0, tempY=0, tempOrientation=theta/(2*numberOfIntermediatePoints);

			double l = (2 * radiusOfCurvature *sinf(theta/(2*numberOfIntermediatePoints)));

			for(int j = 0; j < numberOfIntermediatePoints; ++j){
				tempX += l*sinf(tempOrientation);
				tempY += l*cosf(tempOrientation);
				tempOrientation += theta/numberOfIntermediatePoints;

				fprintf(seedOutput,"%lf %lf\n", -tempX, tempY);
			}

		}

		//generating straight seed
		velocityRatio = 1.0;
		finalX = 0;
		finalY = (2*numberOfSeeds+1)*MULTIPLIER;
		finalOrientation = M_PI/2;
		
		fprintf(seedOutput, "%lf %lf %lf %lf\n", velocityRatio, finalX, finalY, finalOrientation*180/M_PI);
		
		numberOfIntermediatePoints = (finalX + finalY)/10;
		fprintf(seedOutput, "%d\n", numberOfIntermediatePoints);

		for (int i = 1; i <= numberOfIntermediatePoints; ++i){
			fprintf(seedOutput,"%lf %lf\n", finalX, finalY*i/numberOfIntermediatePoints);
		}


		for(int i = 0; i < numberOfSeeds; ++i){
			
			finalX = (i + 1)*MULTIPLIER;
			finalY = (2*numberOfSeeds-i)*MULTIPLIER;

			double radiusOfCurvature = (finalX*finalX + finalY*finalY)/(2 * finalX);
			double theta = asinf (finalY / radiusOfCurvature);

			finalOrientation = M_PI/2 - theta;
			velocityRatio = 1 + distanceBetweenWheels / radiusOfCurvature;
			fprintf(seedOutput, "%lf %lf %lf %lf\n", velocityRatio, -finalX, -finalY, 180 + finalOrientation*180/M_PI);

			numberOfIntermediatePoints = (finalX + finalY)/10;
			fprintf(seedOutput, "%d\n", numberOfIntermediatePoints);

			double tempX=0, tempY=0, tempOrientation=theta/(2*numberOfIntermediatePoints);

			double l = (2 * radiusOfCurvature *sinf(theta/(2*numberOfIntermediatePoints)));

			for(int j = 0; j < numberOfIntermediatePoints; ++j){
				tempX += l*sinf(tempOrientation);
				tempY += l*cosf(tempOrientation);
				tempOrientation += theta/numberOfIntermediatePoints;

				fprintf(seedOutput,"%lf %lf\n", -tempX, -tempY);
			}

		}

		for(int i = 0; i < numberOfSeeds; ++i){
			
			finalX = (i + 1)*MULTIPLIER;
			finalY = (2*numberOfSeeds-i)*MULTIPLIER;

			double radiusOfCurvature = (finalX*finalX + finalY*finalY)/(2 * finalX);
			double theta = asinf (finalY / radiusOfCurvature);

			finalOrientation = M_PI/2 - theta;
			velocityRatio = 1 + distanceBetweenWheels / radiusOfCurvature;

			fprintf(seedOutput, "%lf %lf %lf %lf\n", velocityRatio, finalX, -finalY, 360 - finalOrientation*180/M_PI);

			numberOfIntermediatePoints = (finalX + finalY)/10;
			fprintf(seedOutput, "%d\n", numberOfIntermediatePoints);

			double tempX=0, tempY=0, tempOrientation=theta/(2*numberOfIntermediatePoints);

			double l = (2 * radiusOfCurvature *sinf(theta/(2*numberOfIntermediatePoints)));

			for(int j = 0; j < numberOfIntermediatePoints; ++j){
				tempX += l*sinf(tempOrientation);
				tempY += l*cosf(tempOrientation);
				tempOrientation += theta/numberOfIntermediatePoints;

				fprintf(seedOutput,"%lf %lf\n", tempX, -tempY);
			}

		}
		//generating straight seed
		velocityRatio = 1.0;
		finalX = 0;
		finalY = (2*numberOfSeeds+1)*MULTIPLIER;
		finalOrientation = M_PI/2;
		
		fprintf(seedOutput, "%lf %lf %lf %lf\n", velocityRatio, finalX, -finalY, finalOrientation*180/M_PI);
		
		numberOfIntermediatePoints = (finalX + finalY)/10;
		fprintf(seedOutput, "%d\n", numberOfIntermediatePoints);

		for (int i = 1; i <= numberOfIntermediatePoints; ++i){
			fprintf(seedOutput,"%lf %lf\n", finalX, -finalY*i/numberOfIntermediatePoints);
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