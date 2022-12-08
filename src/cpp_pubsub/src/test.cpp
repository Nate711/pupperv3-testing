#include <stdio.h>
#include <array>
#include <bit>
#include <iostream>
#include <vector>
#include "motor_controller_node.hpp"

/* function to show bytes in memory, from location start to start+n*/
void show_mem_rep(char *start, int n) {
  int i;
  for (i = 0; i < n; i++)
    printf(" %.2x", start[i]);
  printf("\n");
}

/*Main function to call above function for 0x01234567*/
int main() {
  int i = 0x01234567;
  show_mem_rep((char *)&i, sizeof(i));
  // SHOWS LITTLE ENDIANNESS
  // LSB is data[0]

  std::vector<double> input = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
  auto res = MotorControllerNode::split_vector(input);
  for (auto arr : res) {
    for (auto item : arr) {
      std::cout << item << " ";
    }
    std::cout << std::endl;
  }

  return 0;
}