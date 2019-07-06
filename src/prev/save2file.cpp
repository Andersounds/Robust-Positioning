// HOW TO USE.
/*
- DEFINE FILE-OBJECTS
std::ofstream file_true;
std::ofstream file_estimated;
// BEFORE/START
file_true.open("NAME.txt", std::ios::out | std::ios::app);
// EXAMLPE
std::vector<std::vector <float>> input{xPath,yPath,zPath,phi};
build_path(input,file_true);
// AT END-CLOSE FILE
file_true.close();
*/
// For writing and reading files
#include <iostream>
#include <fstream>

// Converting float into string
template <typename T> std::string tostr(const T& t) {
   std::ostringstream os;
   os<<t;
   return os.str();
}

// Adding a row of values ROW, into the file PATH
void build_row(std::vector<float> row, std::ofstream& path){
  int n_rows = row.size();
  if(path.is_open()){
      for (int i = 0; i < n_rows; i++){
      path << tostr(row[i]);
        if(i<(n_rows-1)){
          path << ",";
        }
      }
    path <<"\n";
  }
}

// Building the whole .txt file containing whole path
void build_path(std::vector<std::vector <float>> values,std::ofstream& path){
  int len = values[0].size();
  int n_elements = values.size();

  for(int i = 0; i < len; i++){
    std::vector<float> row;
    for(int j = 0; j < n_elements; j++){
      //float a = values[j][i];
      row.push_back(values[j][i]);
    }
    build_row(row, path);
  }
}
