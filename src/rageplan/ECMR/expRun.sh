#!/bin/sh

### Problem definition
problem=hotel #Problem name: rocksample, cellar, etc.

### Algorithm parameters
hotel=$1
policy=$2 #R = RAGE, P = POMCP

if [ $policy = R ]; then
    rolloutKnowledge=3 # 1 = random, 2 = preferred actions, 3 = PGS
    fTable=0 #IRE y/n
else
    rolloutKnowledge=1 # 1 = random, 2 = preferred actions, 3 = PGS
    fTable=0 #IRE y/n
fi

minDoubles=$3
maxDoubles=$4

runs=100
numSteps=100
verbose=0
timeout=100000
fTable=0
inputFile=insectHotel$hotel.prob
outputFile=$problem.$policy.$minDoubles-$maxDoubles"_$hotel".out


### Run RAGE
../rageH --problem $problem --inputFile $inputFile --minDoubles $minDoubles --maxDoubles $maxDoubles --numSteps $numSteps --runs $runs --rolloutKnowledge $rolloutKnowledge --fTable $fTable --verbose $verbose --timeout $timeout --outputFile $outputFile &
