#! /bin/bash

rosrun quest run_seq_client \
       \
       /seq/negthrust.bin \
       /seq/attrate.bin \
       /seq/vislam_on.bin \
       /seq/log_on.bin \
       /seq/arm.bin \
       \
       /seq/linvel_updown.bin \
       \
       /seq/negthrust.bin \
       /seq/attrate.bin \
       /seq/log_off.bin \
       /seq/vislam_off.bin \
       /seq/disarm.bin
