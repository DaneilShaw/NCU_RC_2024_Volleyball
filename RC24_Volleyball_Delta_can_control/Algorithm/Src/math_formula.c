#include "math_formula.h"

/**************************************************************
	@brief:		限幅函数
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
void limit_amplitude(float *a, float *limit)
{
	if(*a > *limit)
		*a = *limit;
	else if(*a < -*limit)
		*a = -*limit;
}

/**************************************************************
	@brief:		牛顿法求平方根值
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
float InvSqrt(float x) 
{
 	float xhalf = 0.5f*x; 	int i = *(int*)&x; // get bits for floating VALUE 	
	i = 0x5f375a86- (i>>1); // gives initial guess y0 	
	x = *(float*)&i; // convert bits BACK to float 	
	x = x*(1.5f-xhalf*x*x); // Newton step, repeating increases accuracy 	
	x = x*(1.5f-xhalf*x*x); // Newton step, repeating increases accuracy 	
	x = x*(1.5f-xhalf*x*x); // Newton step, repeating increases accuracy 	
	return 1/x; 
}

/**************************************************************
	@brief:		冒泡排序法
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
int Bubble_Sort(int data[], int n)
{
	for(int i=0;i<n-1;i++)//这里是外循环，有n个数就比较n-1次
 {
	 for(int j=0;j<n-1-i;j++)//这是内循环，每轮比较n-1-（上一轮的除去的数的个数）
	 {
		 if(data[j]>data[j+1])//判断如果左边的数大于右边的数，就把大的数往右移
     {
			 //这个是交换不用第三方变量的方式，当然也可以借用第三方变量来交换
       data[j]=data[j]+data[j+1];
       data[j+1]=data[j]-data[j+1];
       data[j]=data[j]-data[j+1];
		 }
	 }
 }
 return data[0];
}


/**************************************************************
	@brief:		绝对值函数
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
float absoluteValue(float x)
{
    if (x < 0)	return -x;
    else 	return x;
}


