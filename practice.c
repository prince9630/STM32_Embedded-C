/******************************************************************************

                            Online C Compiler.
                Code, Compile, Run and Debug C program online.
Write your code in this editor and press "Run" button to compile and execute it.

*******************************************************************************/
/*******************************************************************************
  * File Name          : practice.c
  * Description        : find out maximum number among 3 numbers enter by users
  * 
  * Author:              Prince Patel
  * Date:                2022/10/19				 
  ******************************************************************************
  */

#include <stdio.h>



int maxNumber (int number1, int number2, int number3);
int main()

{
    int num1;
    int num2;
    int num3;
    
    printf("enter three number:\n");                     //to enter three numbers by users
    scanf("%d %d %d", &num1, &num2, &num3);
    
    int max = maxNumber(num1, num2, num3);
    printf("maximum number is: %d", max);

    return 0;
}
// FUNCTION      : if-else ladder
// DESCRIPTION   : it will allow to check between multiple expression
//   
// PARAMETERS    :
//   <list the parameters passed to function and what they do>
//
// RETURNS       :
//   <if the function returns something describe it here>
int maxNumber (int number1, int number2, int number3)
{
    
    if(number1 > number2 & number1 > number3 )                        //compare number 1 with number 2 and 3
    {
        return number1;                                               //if number 1 is greater than other numbers then number1 will be display
    }
    
    else if(number2 > number1 & number2 > number3 )                   //compare number 2 with number 1 and 3
    {
        return number2;                                               //if number 2 is greater than other numbers then number2 will be display
    }
    else
    {
        return number3;                                              //otherwise number3 will be display 
    }
}
