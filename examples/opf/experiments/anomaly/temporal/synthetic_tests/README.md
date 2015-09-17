=== Synthetic dataset benchmarks for NuPIC anomaly detection ===

1/ Run:
cd $NUPIC/examples/opf/experiments/anomaly/temporal/
$NUPIC/scripts/run_opf_experiment.py synthetic_tests/

2/ Modify:
follow the header format in `header.txt`, eventually update `description.py`. Data is expected to be in 
`data.csv` file. 
You can run the script `datasets/generate_data.m` to genereta and write new data to CSV. Or just combine existing data (sine, random, ...),
eg using linux tools: `cat a.csv >> data.csv` appends a.csv to the end of the data.csv file. `head -n 100 a.csv >> data.csv` appends the first 
100 rows from A to the end of Data. 

