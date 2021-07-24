#include <string>
#include <fstream>
#include <vector>
#include <utility> // std::pair

#define MAX_SENSOR_SAMPLE 10000
const float LO = 5.0, HI = 50.0;
void write_csv(std::string filename, std::vector<std::pair<std::string, std::vector<float>>> dataset){
    // Create an output filestream object
    std::ofstream myFile(filename);
    
    // Send column names to the stream
    for(int j = 0; j < dataset.size(); ++j)
    {
        myFile << dataset.at(j).first;
        if(j != dataset.size() - 1) myFile << ","; // No comma at end of line
    }
    myFile << "\n";
    
    // Send data to the stream
    for(int i = 0; i < dataset.at(0).second.size(); ++i)
    {
        for(int j = 0; j < dataset.size(); ++j)
        {
            myFile << dataset.at(j).second.at(i);
            if(j != dataset.size() - 1) myFile << ","; // No comma at end of line
        }
        myFile << "\n";
    }
    
    // Close the file
    myFile.close();
}

inline float random_reading(void){
    return LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
}

int main() {
    std::vector<float> vec1;
    std::vector<float> vec2;
    
    for(auto i = 0 ; i < MAX_SENSOR_SAMPLE ; i++){
        vec1.push_back(random_reading());
        vec2.push_back(random_reading());
    }

    // Wrap into a vector
    std::vector<std::pair<std::string, std::vector<float>>> vals = {{"S1 Distance from wall", vec1}, {"S2 Distance from wall", vec2}};
    
    // Write the vector to CSV
    write_csv("sensorData.csv", vals);
    
    return 0;
}
