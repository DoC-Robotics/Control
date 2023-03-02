#!/usr/bin/env python
# By Jacek Zienkiewicz and Andrew Davison, Imperial College London, 2014
# Based on original C code by Adrien Angeli, 2009

import random
import os
import time     # import the time library for the sleep function
import brickpi3  # import the BrickPi3 drivers
import math
import statistics
BP = brickpi3.BrickPi3()
BP.reset_all()
import numpy as np
'''
NOTES:
PLease check function characterize location

'''

def initialize_brickpi():
    left_motor = BP.PORT_B
    right_motor = BP.PORT_C
    BP.set_motor_limits(left_motor, 50, 200)
    BP.set_motor_limits(right_motor, 50, 200)
    ultrasonic_sensor = BP.PORT_2
    BP.set_sensor_type(ultrasonic_sensor, BP.SENSOR_TYPE.NXT_ULTRASONIC)


    #   reset motor states


left_motor = BP.PORT_B
right_motor = BP.PORT_C
ultrasonic_sensor = BP.PORT_2


bin_number = 36
# Location signature class: stores a signature characterizing one location
class LocationSignature:
    def __init__(self, no_bins = 36):
        self.sig = [0] * no_bins
        
    def print_signature(self):
        for i in range(len(self.sig)):
            print(self.sig[i])

# --------------------- File management class ---------------
class SignatureContainer():
    def __init__(self, size = 5):
        self.size      = size; # max number of signatures that can be stored
        self.filenames = [];
        
        # Fills the filenames variable with names like loc_%%.dat 
        # where %% are 2 digits (00, 01, 02...) indicating the location number. 
        for i in range(self.size):
            self.filenames.append('loc_{0:02d}.dat'.format(i))

    # Get the index of a filename for the new signature. If all filenames are 
    # used, it returns -1;
    def get_free_index(self):
        n = 0
        while n < self.size:
            if (os.path.isfile(self.filenames[n]) == False):
                break
            n += 1
            
        if (n >= self.size):
            return -1;
        else:    
            return n;
 
    # Delete all loc_%%.dat files
    def delete_loc_files(self):
        print("STATUS:  All signature files removed.")
        for n in range(self.size):
            if os.path.isfile(self.filenames[n]):
                os.remove(self.filenames[n])
            
    # Writes the signature to the file identified by index (e.g, if index is 1
    # it will be file loc_01.dat). If file already exists, it will be replaced.
    def save(self, signature, index):

        #writes the signature to a file
        filename = self.filenames[index]
        if os.path.isfile(filename):
            os.remove(filename)
            
        f = open(filename, 'w')

        for i in range(len(signature.sig)):
            s = str(signature.sig[i]) + "\n"
            f.write(s)
        f.close();

    # Read signature file identified by index. If the file doesn't exist
    # it returns an empty signature.
    def read(self, index):
        ls = LocationSignature()
        filename = self.filenames[index]
        if os.path.isfile(filename):
            f = open(filename, 'r')
            for i in range(len(ls.sig)):
                s = f.readline()
                if (s != ''):
                    ls.sig[i] = int(s)
            f.close();
        else:
            print("WARNING: Signature does not exist.")
        
        return ls
        
def characterize_location(ls):
    '''
    This takes a class of ls corresponding to a signature container. 
    The function primarily will update the array of signature ls.sig[] and  
    '''
    #rotates by rotation amount each times
    rotation_scale_map = 222.0/90.0
    amount_per_rotation = 360/len(ls.sig)*(rotation_scale_map)


    #reset 
    try:
        BP.offset_motor_encoder( left_motor, BP.get_motor_encoder(left_motor)) 
        BP.offset_motor_encoder( right_motor, BP.get_motor_encoder(right_motor)) 
    except IOError as error:
        print(error)

    print("In function - robot movement")
    print("rotation START")

    for i in range(0,len(ls.sig)):
        #generates between 0 and 255. 
        #reset 
        try:
            BP.offset_motor_encoder( left_motor, BP.get_motor_encoder(left_motor)) 
            BP.offset_motor_encoder( right_motor, BP.get_motor_encoder(right_motor)) 
        except IOError as error:
            print(error)

        #set the rotation amount towards left.
        BP.set_motor_position(left_motor,-amount_per_rotation)
        BP.set_motor_position(right_motor,amount_per_rotation)
        while (BP.get_motor_status(left_motor)[2] < -amount_per_rotation): 
            time.sleep(0.1)


        ls.sig[i] = distance_measured()
        print("location signature",ls.sig)

        time.sleep(0.1)
    #complete 1 more rotation I think TOCHECK!!!!!
    #BP.set_motor_position(left_motor,-amount_per_rotation)
    #BP.set_motor_position(right_motor,amount_per_rotation)

    print("loop exit")
    pass
# FILL IN: compare two signatures
def compare_signatures(ls1, ls2):
    curr_diff = 0
    print("TODO:    You should implement the function that compares two signatures.")
    for i in range(len(ls1.sig)):
        signature_1 = ls1.sig[i]
        signature_2 = ls2.sig[i]
        curr_diff += (signature_2-signature_1)**2

    return curr_diff


def distance_measured():
    '''
    Deduce the total distance to wall as measured by the ultrasound sensors. 
    '''
    #time will tick by
    curr_time = time.time()
    elapse_time = 0
    while elapse_time<1.0:
        readings = []
        try:
            #will 
            ultrasonicState = BP.get_sensor(ultrasonic_sensor)
            readings.append(ultrasonicState)
            time.sleep(0.1)

        except:
            BP.reset_all()
            initialize_brickpi()
            time.sleep(0.1)
        #get the elapsed time
        elapse_time = time.time()-curr_time

    if len(readings) == 0:
        #retake measurements
        return distance_measured()
    else:
        median_reading = statistics.median(readings)
        print("Real reading to the facing wall: ", median_reading)
        # NO OBSTACLE AVOIDANCE, AS NOT NEEDED IN THE GIVEN PATH
        # median_reading only used for the updating of MCL particles
        return median_reading




def learn_location():
    ''' 
    This function characterizes the current location, and stores the obtained signature into the next available file.
    '''
    #take a total of 30 signatures starting from angle of 0 degrees. 
    ls = LocationSignature(no_bins = bin_number)
    characterize_location(ls)
    idx = signatures.get_free_index();
    if (idx == -1): # run out of signature files
        print("\nWARNING:")
        print("No signature file is available. NOTHING NEW will be learned and stored.")
        print("Please remove some loc_%%.dat files.\n")
        return
    
    #writes location signature to an index file that is free. 
    signatures.save(ls,idx)
    print("STATUS:  Location " + str(idx) + " learned and saved.")

def convert_signature_to_hist(location,mode='normal'):
    '''
    
    Will take a signature with values between 0-256 and then correspondingly convert the signature to a depth based histogram.

    '''
    bin_count = 256//5 #determines number of bins based on max depth of 256 
    depth_hist = [0 for i in range(bin_count)]

    print("debug location",location)
    print("debug signature",location.sig)
    
    if (mode=="normal"):
        loc_sig = location.sig
    else:
        loc_sig = location 


    for val in loc_sig:
        #gets the relevant value in the histogram
        bin = val//5
        #not fastest way of doing things but can prevent errors. 
        curr_bin_val = depth_hist[bin]
        depth_hist[bin] = curr_bin_val+1
    
    #will return a depth histogram. 
    return depth_hist





# This function tries to recognize the current location.
# 1.   Characterize current location
# 2.   For every learned locations
# 2.1. Read signature of learned location from file
# 2.2. Compare signature to signature coming from actual characterization
# 3.   Retain the learned location whose minimum distance with
#      actual characterization is the smallest.
# 4.   Display the index of the recognized location on the screen
def recognize_location():
    '''
    This function will compare the current signature to all signatures whcih are present and will determine accordingly which signature the location most accurately corresponds to. 
    '''
    ls_obs = LocationSignature(no_bins = bin_number);
    characterize_location(ls_obs);
    min_dist = float('inf')
    best_idx = 0
    # FILL IN: COMPARE ls_read with ls_obs and find the best match
    for idx in range(signatures.size):
        print("STATUS:  Comparing signature " + str(idx) + " with the observed signature.")
        ls_read = signatures.read(idx).sig;#reads a file which can be converted to histograms. 
        
        print(ls_read)
        #converts the angle against depth values to depth histograms with a depth of 5 per bin. 
        ls_read = convert_signature_to_hist(ls_read,mode='diff')
        print("line 278 reached")
        ls_obs = convert_signature_to_hist(ls_obs)

        #gets the squared difference for every location. 
        dist    = compare_signatures(ls_obs, ls_read)

        #save and denote the best locations. 
        if dist<min_dist:
            min_dist = dist
            best_idx = idx
    
    #return 
    print("comparison completed, best index:",best_idx,
          "Best Distance:",min_dist)
    
    return best_idx

       
       
            

# Prior to starting learning the locations, it should delete files from previous
# learning either manually or by calling signatures.delete_loc_files(). 
# Then, either learn a location, until all the locations are learned, or try to
# recognize one of them, if locations have already been learned.

signatures = SignatureContainer(5);
#signatures.delete_loc_files()

learn_location();
recognize_location();

