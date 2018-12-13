You require OpenCV (C++) to run the code

To run homotopy constrained A-star planner:
  1. In main.cpp - ensure #include"planner4.h" is uncommented and #include "mha*.h" is commented
  2.  In planner4.h set goal->x and goal->y and goal->signatures
  3. To compile: g++ main.cpp `pkg-config --cflags --libs opencv` --std=c++11


To run homotopy constrained MHA-star planner:
  1. In main.cpp - ensure #include "mha*.h" is uncommented and #include "planner4.h" is uncommented
  2.  In mha*.h set goal->x and goal->y and goal->signatures
  3. To compile: g++ main.cpp `pkg-config --cflags --libs opencv` --std=c++11

For trajectory optimization:
The cost map is inputted in the form of a text file. Which is read and the output path is written to a file
  1. In Gradient_descent_generation.cpp - In planner4.h uncomment lines 58,59,64,75 (These lines have been commented as
  planner4.h is used for point robot planning and there's no need for the path to be written onto a file)
  2.  In planner4.h set goal->x and goal->y and goal->signatures
  3. To compile: g++ Gradient_descent_generation.cpp `pkg-config --cflags --libs opencv` --std=c++11
This writes the output path onto a text file, which is used by the trajectory optimiser
