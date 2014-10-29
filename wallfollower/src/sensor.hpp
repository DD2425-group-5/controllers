#ifndef SENSOR_HPP
#define SENSOR_HPP
class sensor {
public:
	sensor(){}
	sensor(int num);
	double get_distance();
	int get_value();
private:
	double distance;
	int value;
	int number;
};
#endif