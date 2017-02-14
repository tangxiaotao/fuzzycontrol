/* 
 * Mobile robot trajectory tracking using fuzzy inference
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <math.h>

/* handy macros */
#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))
#define NUM_RULES	15

/* the domin of Ld */
double Ld[4] = {0, 1/3.0, 2/3.0, 1};

/*the fuzzy rules
 *Z is the 1
 ×S is the 2
 *M is the 3
 *L is the 4
 */
int rule[5][3] = {{0,1,2},{1,2,2},{2,2,3},{1,2,2},{0,1,2}};

/* membership wedges */
enum
{
	/* the membership wedges of distance */
	NLd = 10,
	NSd = 11,
	Zd  = 12,
	PSd = 13,
	PLd = 14,
	
	/* the membership wedges of velocity */
	Zv = 20,
	Sv = 21,
	Lv = 22
};

typedef struct 
{
	double value;	/* crisp value */
	int membership_func;    /* the specific function Small, Large ... */
}fuzzy_in;


/* enum specifying the type of membership function */


/* aggregate the antecedents and compute the weight (membership degree) */
double compute_aggregation(double memb_val_in1, double memb_val_in2)
{
	return min(memb_val_in1, memb_val_in2);
}

/* compute the membership vf for a crisp input */
double compute_membership(fuzzy_in* in)
{
	double a, b, c, d, sigma, mean;
	double out = 0.0f;
	switch(in->membership_func){
		/* for distance error*/
		case NLd:
			a = -1.5, b = -1, c = -0.5;
		break;
		case NSd:
			a = -1, b = -0.5, c = 0;
		break;
		case Zd:
			a = -0.5, b = 0, c = 0.5;
		break;
		case PSd:
			a = 0, b = 0.5, c = 1;
		break;
		case PLd:
			a = 0.5, b = 1, c = 1.5;
		break;
		case Zv:
			a = -0.4, b = 0, c = 0.4;
		break;
		case Sv:
			a = 0.1, b = 0.5, c = 0.9;
		break;
		case Lv:
			a = 0.6, b = 1, c = 1.4;
		break;
	}
	
	/*s使用等腰三角形为隶属度函数*/
	out = max(min((in->value - a)/(b-a), (c - in->value)/(c - b)), 0);			
	return out; 
}

double fuzzyOutput(double d, double v)
{
	fuzzy_in *d_in = (fuzzy_in*)calloc(1, sizeof(fuzzy_in));
	fuzzy_in *v_in = (fuzzy_in*)calloc(1, sizeof(fuzzy_in));
	d_in->value = d;
	v_in->value = v;
	
	double w[NUM_RULES] = {0.0f};
	double u[NUM_RULES] = {0.0f};

	int k = 0;
	double sum_w = 0.0f, sum_u = 0.0f;
	for (int i=10; i<=14; i++) {
		for (int j=20; j<=22; j++) {
			d_in->membership_func = i;
			v_in->membership_func = j;
			w[k] = compute_aggregation(compute_membership(d_in), compute_membership(v_in));
			//printf("%f\n",compute_membership(d_in));
			//printf("%f\n",compute_membership(v_in));
			u[k] = Ld[rule[i-10][j-20]];
			//printf("%f\n",Ld[rule[i-10][j-20]]);
			//printf("%f\n",u[k]);
			sum_w += w[k];
			sum_u += w[k]*u[k];
			k++;
			//printf("\n");
		}
	}
	//printf("sum_w = %f\n",sum_w);
	/*采用面积重心法 */
	double out = sum_u/sum_w;
	
	free(d_in);
	free(v_in);
	return out;
}

int main() {
	double d = 0.5;
	double v = 0.3;
	double u = fuzzyOutput(d, v);
	printf("%f\n",u);
}
