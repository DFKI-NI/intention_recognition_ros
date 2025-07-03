#!/bin/sh

### Problem definition
problem=hotel #Problem name: rocksample, cellar, etc.
inputFile=hotel.prob

### Algorithm parameters
policy=R #R = RAGE, P = POMCP
mode=E #D = debug, E = experiment

if [ $policy = R ]; then
    rolloutKnowledge=3 # 1 = random, 2 = preferred actions, 3 = PGS
    fTable=0 #IRE y/n
else
    rolloutKnowledge=1 # 1 = random, 2 = preferred actions, 3 = PGS
    fTable=0 #IRE y/n
fi

### Search and experiment parameters
numSteps=100

if [ $mode = D ]; then
    minDoubles=15
    maxDoubles=15
    runs=1
    verbose=1
    timeout=1000
    outputFile=output_$policy.txt
else
    minDoubles=16
    maxDoubles=16
    runs=100
    verbose=0
    timeout=5000
    outputFile=$problem.$policy.$minDoubles-$maxDoubles.out
fi

### Run RAGE
./rageH2 --problem $problem --inputFile $inputFile --minDoubles $minDoubles --maxDoubles $maxDoubles --numSteps $numSteps --runs $runs --rolloutKnowledge $rolloutKnowledge --fTable $fTable --verbose $verbose --timeout $timeout --outputFile $outputFile
