#!/bin/bash
# formats output from CLA's inference
# ./formatOutput.sh inferenceResultsFile.csv
# 2 = actual/raw value
# 7 = anomaly score

#1 get rid of the {12 232 232} field that breaks "normal" CSV 
# cut only the ecg (orig value) and pred.1 (prediction for T+1)
cat "$1" | sed -e's/{.*}/X/g' | sed -e's/\[.*\]/Y/g' | cut -d',' -f7 

