#include <stdio.h>
#include <cmath>
#include <algorithm>

#define MULTIPLIER 1

int main()
{
	int numberOfSeeds, numberOfIntermediatePoints;
	double gridSpacing, velocityRatio, finalX, finalY, finalOrientation, costOfSeed;
	double intermediateXValue, intermediateYValue;

	FILE *seedOutput,readInput;
	
	seedOutput=fopen("./test_seeds.txt","w");
	
	//printf("OutputFormat:\nnumberOfSseeds\nvelocityRatio finalX finalY finalOrientation costOfSeed\nnSeedPoints\nintermediateXValue intermediateYValue\n");
	
	printf("Enter number of seeds to be generated per quadrant\n");
	scanf("%d", &numberOfSeeds);

	fprintf(seedOutput, "%d\n", 2*numberOfSeeds+1);

	for(int i = 0; i < numberOfSeeds; ++i){
		
		finalX = (i + 1)*MULTIPLIER;
		finalY = (2*numberOfSeeds-i)*MULTIPLIER;
		finalOrientation = atanf( finalY / finalX);

		double radiusOfCurvature = (finalX*finalX + finalY*finalY)/(2 * finalX);
		double theta = (M_PI - 2 * finalOrientation);

		velocityRatio = 0;
		costOfSeed = finalX + finalY - finalOrientation;

		fprintf(seedOutput, "%lf %lf %lf %lf %lf\n", velocityRatio, finalX, finalY, finalOrientation*180/M_PI, costOfSeed);

		numberOfIntermediatePoints = (finalX + finalY)/MULTIPLIER;
		fprintf(seedOutput, "%d\n", numberOfIntermediatePoints);

		double tempX=0, tempY=0, tempOrientation=theta/(2*numberOfIntermediatePoints);

		float l = (2 * radiusOfCurvature *sinf(theta/(2*numberOfIntermediatePoints)));

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
		finalOrientation = atanf( finalY / finalX);

		double radiusOfCurvature = (finalX*finalX + finalY*finalY)/(2 * finalX);
		double theta = (M_PI - 2 * finalOrientation);

		velocityRatio = 0;
		costOfSeed = finalX + finalY - finalOrientation;

		fprintf(seedOutput, "%lf %lf %lf %lf %lf\n", velocityRatio, -finalX, finalY, finalOrientation*180/M_PI, costOfSeed);

		numberOfIntermediatePoints = (finalX + finalY)/MULTIPLIER;
		fprintf(seedOutput, "%d\n", numberOfIntermediatePoints);

		double tempX=0, tempY=0, tempOrientation=theta/(2*numberOfIntermediatePoints);

		float l = (2 * radiusOfCurvature *sinf(theta/(2*numberOfIntermediatePoints)));

		for(int j = 0; j < numberOfIntermediatePoints; ++j){
			tempX += l*sinf(tempOrientation);
			tempY += l*cosf(tempOrientation);
			tempOrientation += theta/numberOfIntermediatePoints;

			fprintf(seedOutput,"%lf %lf\n", -tempX, tempY);
		}

	}

	velocityRatio = 0;
	finalX = 0;
	finalY = (2*numberOfSeeds+1)*MULTIPLIER;
	finalOrientation = M_PI/2;
	costOfSeed = finalY*0.75;
	fprintf(seedOutput, "%lf %lf %lf %lf %lf\n", velocityRatio, finalX, finalY, finalOrientation*180/M_PI, costOfSeed);
	numberOfIntermediatePoints = (finalX + finalY)/MULTIPLIER;
	
	for (int i = 0; i < numberOfIntermediatePoints; ++i)
	{
		fprintf(seedOutput,"%lf %lf\n", finalX, finalY*i/numberOfIntermediatePoints);
	}
	return 0;
}