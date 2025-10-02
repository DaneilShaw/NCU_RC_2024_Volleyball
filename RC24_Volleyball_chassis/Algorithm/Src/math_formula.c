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
int Bubble_Sort(float data[], int n)
{
	float buf[n];
	memcpy(buf, data, n * sizeof(float));//将data的数据复制到buf中
	/*通过冒泡排序法获得最大数据*/
	for(int i = 0; i < n - 1; i++)//这里是外循环，有n个数就比较n-1次
	{
	 for(int j = 0; j < n - 1 - i; j++)//这是内循环，每轮比较n-1-（上一轮的除去的数的个数）
	 {
		 if(data[j] > data[j+1])//判断如果左边的数大于右边的数，就把大的数往右移
     {
			 //使用临时变量来交换
       float temp = data[j];
       data[j] = data[j+1];
       data[j+1] = temp;
		 }
	 }
	}
  int x = 0;
  for(int i = 0; i < n; i++)//找到原数组中，最大数据的次序
  {
		if(buf[i] == data[n-1])
		{
			x = i;
		  break;
	  }
  }
  return x;
}


