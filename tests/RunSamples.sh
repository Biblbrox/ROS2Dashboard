#!/bin/bash

python3 SamplePublisherString.py &
./SamplePublisherWebcam.sh &
./SampleSubscriberWebcam.sh &

