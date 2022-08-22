#include <stdio.h>
#include <iostream>
#include <bit>
#include <array>

/* function to show bytes in memory, from location start to start+n*/
void show_mem_rep(char *start, int n)
{
    int i;
    for (i = 0; i < n; i++)
        printf(" %.2x", start[i]);
    printf("\n");
}
struct MotorData
{
    uint8_t error = 0;
    uint8_t motor_id = 0;
    float multi_angle = 0;
};

class ArrayThing {
    public:
        std::array<std::array<MotorData,4>,4> data;
        ArrayThing() {
            data.at(0).at(0) = MotorData{0, 1, 50.0};
        }
        std::array<std::array<MotorData,4>,4> getter() {
            return data;
        }
};

/*Main function to call above function for 0x01234567*/
int main()
{
    int i = 0x01234567;
    show_mem_rep((char *)&i, sizeof(i));
    // SHOWS LITTLE ENDIANNESS
    // LSB is data[0]

    ArrayThing thing;
    auto data_copy = thing.getter();
    data_copy.at(0).at(0).multi_angle = 20.0;

    std::cout << "Copy: " << data_copy.at(0).at(0).multi_angle << std::endl;
    std::cout << "Object original: " << thing.data.at(0).at(0).multi_angle << std::endl;
    std::cout << sizeof(data_copy) << std::endl;
    std::cout << sizeof(MotorData) << std::endl;
    std::cout << sizeof(float) << std::endl;
    std::cout << sizeof(std::array<float,4>{0,0,0,0}) << std::endl;
    return 0;
}