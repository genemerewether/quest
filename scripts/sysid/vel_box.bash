#! /bin/bash

rosrun quest run_seq_client \
       /seq/takeoff.bin \
       /seq/linvel_box.bin \
       /seq/landing.bin
