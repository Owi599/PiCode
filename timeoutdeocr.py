import timeout_decorator
import time
import random

@timeout_decorator.timeout(3)  # Set a timeout of 3 seconds
def randomFunction(n):
    # Simulating a function that might take too long
    time.sleep(n)  # Sleep for 5 seconds
    return "Function completed successfully"


while True:
    n = random.uniform(0.01, 5)
    try:
        result = randomFunction(n)
        print(result)
    except timeout_decorator.TimeoutError:
        print("Function timed out after timeout time reached")
        continue

# timeoutdeocr.py
# This code demonstrates how to use the timeout_decorator to limit the execution time of a function.
# The randomFunction is designed to sleep for 5 seconds, but the timeout decorator will raise a TimeoutError after 3 seconds.
# The try-except block catches this error and prints a message indicating that the function timed out.
# Note: Make sure to install the timeout_decorator package using pip if you haven't already:
# pip install timeout-decorator
# This code is a simple demonstration of how to use the timeout_decorator package to limit the execution time of a function.
# You can modify the timeout duration and the function logic as needed.     