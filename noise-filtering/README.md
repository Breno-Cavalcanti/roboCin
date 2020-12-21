# Noise filtering:

To run this filter you will need to have:
- [grSim](https://github.com/RoboCup-SSL/grSim)
- [SSL client](https://github.com/robocin/ssl-client)
warning: you will need to get a preverius version of the ssl client, because this project was develop using a preverious version and I was not able to test in the actual version.

Library versions:
- pandas: 1.0.5
- matplotlib: 3.2.2
- seaborn: 0.10.1
- numpy: 1.18.5

# How to test:
- you have 2 options, the first is to run the `validation.ipynb` file and see the results by yourself with the cvs file alredy provied. The anothe option is to create a .csv, to do so, follow the next steps:
  1. Copy the `main.cpp` of this project and paste(to replace) the main.ccp in the `./src`.
  2. open the `client` in the `../grSim/bin` and put all robots out of field and off, except the blue robot with id equal to 0, you can put him in every place.
  3. in the function `update_csv_file`(in the main.cpp) there is the name of the file .csv, you need to create one with this name(or any name, but you need to change in the function) to see the results.
  4. Run the qtProject and use the `client` to change the velocity of the robot.
  5. After that, get the .csv file and put in the same folder with the file `validation2.ipynb`, in the second cell replace `positions_with_noise.csv` for the name of the csv file which you created and see the results.