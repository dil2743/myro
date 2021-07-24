#include "readCsv.hh"

using namespace std;


std::vector<std::pair<std::string, std::vector<float>>> read_csv(std::string filename){
    // Reads a CSV file into a vector of <string, vector<float>> pairs where
    // each pair represents <column name, column values>

    // Create a vector of <string, float vector> pairs to store the result
    std::vector<std::pair<std::string, std::vector<float>>> result;

    // Create an input filestream
    std::ifstream myFile(filename);

    // Make sure the file is open
    if(!myFile.is_open()) throw std::runtime_error("Could not open file");

    // Helper vars
    std::string line, colname;
    float val;

    // Read the column names
    if(myFile.good())
    {
        // Extract the first line in the file
        std::getline(myFile, line);

        // Create a stringstream from line
        std::stringstream ss(line);

        // Extract each column name
        while(std::getline(ss, colname, ',')){
            
            // Initialize and add <colname, float vector> pairs to result
            result.push_back({colname, std::vector<float> {}});
        }
    }

    // Read data, line by line
    while(std::getline(myFile, line))
    {
        // Create a stringstream of the current line
        std::stringstream ss(line);
        
        // Keep track of the current column index
        float colIdx = 0;
        
        // Extract each integer
        while(ss >> val){
            
            // Add the current integer to the 'colIdx' column's values vector
            result.at(colIdx).second.push_back(val);
            
            // If the next token is a comma, ignore it and move on
            if(ss.peek() == ',') ss.ignore();
            
            // Increment the column index
            colIdx++;
        }
    }

    // Close file
    myFile.close();

    return result;
}

int main() {

    std::vector<std::pair<std::string, std::vector<float>>> sensorData = read_csv("sensorData.csv");
    
    for(int i = 0; i < sensorData.at(0).second.size(); ++i)
    {
        for(int j = 0; j < sensorData.size(); ++j)
        {
            cout << sensorData.at(j).second.at(i);
            if(j != sensorData.size() - 1) cout << ","; // No comma at end of line
        }
        cout << "\n";
    }
    return 0;
}
