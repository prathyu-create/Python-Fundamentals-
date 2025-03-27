import math
def is_prime(number):
    if number <=1:
        return False
    for i in range(2,10000):
        if number % i == 0:
         return False
    return True
    # testing
number = 50
if is_prime(number):
    print(f"{number} given number is prime")
else:
    print(f"{number} given number is not prime")


    

