#!/usr/bin/env python
# By Jacek Zienkiewicz and Andrew Davison, Imperial College London, 2014
# Based on original C code by Adrien Angeli, 2009

import random
import os
import time
import brickpi3
import main
import likelihood
import normalising_resampling


BP = brickpi3.BrickPi3()
left_motor = BP.PORT_B
right_motor = BP.PORT_C
ultrasonic_sensor = BP.PORT_2
BP.set_motor_limits(left_motor, 50, 200)
BP.set_motor_limits(right_motor, 50, 200)


bin_number = 15


# Location signature class: stores a signature characterizing one location
class LocationSignature:
    def __init__(self, no_bins=15):
        self.sig = [0] * no_bins

    def print_signature(self):
        for i in range(len(self.sig)):
            print(self.sig[i])


# --------------------- File management class ---------------
class SignatureContainer:
    def __init__(self, size=5):
        self.size = size
        # max number of signatures that can be stored
        self.filenames = []

        # Fills the filenames variable with names like loc_%%.dat
        # where %% are 2 digits (00, 01, 02...) indicating the location number.
        for i in range(self.size):
            self.filenames.append("loc_{0:02d}.dat".format(i))

    # Get the index of a filename for the new signature. If all filenames are
    # used, it returns -1;
    def get_free_index(self):
        n = 0
        while n < self.size:
            if os.path.isfile(self.filenames[n]) == False:
                break
            n += 1

        if n >= self.size:
            return -1
        else:
            return n

    # Delete all loc_%%.dat files
    def delete_loc_files(self):
        print("STATUS:  All signature files removed.")
        for n in range(self.size):
            if os.path.isfile(self.filenames[n]):
                os.remove(self.filenames[n])

    # Writes the signature to the file identified by index (e.g, if index is 1
    # it will be file loc_01.dat). If file already exists, it will be replaced.
    def save(self, signature, index):
        filename = self.filenames[index]
        if os.path.isfile(filename):
            os.remove(filename)

        f = open(filename, "w")

        for i in range(len(signature.sig)):
            s = str(signature.sig[i]) + "\n"
            f.write(s)
        f.close()

    # Read signature file identified by index. If the file doesn't exist
    # it returns an empty signature.
    def read(self, index):
        ls = LocationSignature()
        filename = self.filenames[index]
        if os.path.isfile(filename):
            f = open(filename, "r")
            for i in range(len(ls.sig)):
                s = f.readline()
                if s != "":
                    ls.sig[i] = int(round(float(s)))
            f.close()
        else:
            print("WARNING: Signature does not exist.")

        return ls


# FILL IN: spin robot or sonar to capture a signature and store it in ls
def characterize_location(ls):
    print("TODO:    You should implement the function that captures a signature.")
    rotation_scale_map = 226.0 / 90.0
    amount_per_rotation = 360 / len(ls.sig) * (rotation_scale_map)

    # reset
    try:
        BP.offset_motor_encoder(left_motor, BP.get_motor_encoder(left_motor))
        BP.offset_motor_encoder(right_motor, BP.get_motor_encoder(right_motor))
    except IOError as error:
        print(error)

    print("In function - robot movement")
    print("rotation START")

    for i in range(0, len(ls.sig)):
        # generates between 0 and 255.
        BP.offset_motor_encoder(left_motor, BP.get_motor_encoder(left_motor))
        BP.offset_motor_encoder(right_motor, BP.get_motor_encoder(right_motor))

        # set the rotation amount.
        BP.set_motor_position(left_motor, amount_per_rotation * 1.05)
        BP.set_motor_position(right_motor, -amount_per_rotation * 1.05)
        time.sleep(0.1)
        ls.sig[i] = main.distance_measured()

        time.sleep(0.1)
    print("loop exit")


def convert_signature_to_hist(location):
    """

    Will take a signature with values between 0-256 and then correspondingly convert the signature to a depth based histogram.

    """
    bin_count = 256 // bin_number  # determines number of bins based on max depth of 256
    depth_hist = [0 for i in range(bin_count + 1)]

    for val in location.sig:
        # gets the relevant value in the histogram
        bin = val // bin_number
        depth_hist[int(bin)] += 1

    # will return a depth histogram.
    return depth_hist


# FILL IN: compare two signatures
def compare_signatures(ls1, ls2):
    curr_diff = 0
    print("TODO:    You should implement the function that compares two signatures.")
    for i in range(len(ls1.sig)):
        signature_1 = ls1.sig[i]
        signature_2 = ls2.sig[i]
        curr_diff += (signature_2 - signature_1) ** 2

    return curr_diff


def compare_historgrams(his1, his2):
    curr_diff = 0
    print("TODO:    You should implement the function that compares two histograms.")
    for i in range(len(his1)):
        signature_1 = his1[i]
        signature_2 = his2[i]
        curr_diff += (signature_2 - signature_1) ** 2

    return curr_diff


def find_orientation(ls1, ls2_obs):
    lowest_error_offset = 0
    lowest_error = float("inf")
    for i in range(len(ls1.sig)):
        curr_diff = 0
        for j in range(len(ls1.sig)):
            signature_1 = ls1.sig[j]
            signature_2 = ls2_obs.sig[(j + i) % 15]
            curr_diff += (signature_2 - signature_1) ** 2
        if curr_diff < lowest_error:
            lowest_error = curr_diff
            lowest_error_offset = i
    if lowest_error_offset * -24 < -180:
        return lowest_error_offset * -24 + 360
    else:
        return lowest_error_offset * -24


# This function characterizes the current location, and stores the obtained
# signature into the next available file.
def learn_location():
    ls = LocationSignature()
    characterize_location(ls)
    idx = signatures.get_free_index()
    if idx == -1:  # run out of signature files
        print("\nWARNING:")
        print("No signature file is available. NOTHING NEW will be learned and stored.")
        print("Please remove some loc_%%.dat files.\n")
        return

    signatures.save(ls, idx)
    print("STATUS:  Location " + str(idx) + " learned and saved.")


# This function tries to recognize the current location.
# 1.   Characterize current location
# 2.   For every learned locations
# 2.1. Read signature of learned location from file
# 2.2. Compare signature to signature coming from actual characterization
# 3.   Retain the learned location whose minimum distance with
#      actual characterization is the smallest.
# 4.   Display the index of the recognized location on the screen
def recognize_location(ls_obs, signatures, particles, map):
    best_idx = 0
    characterize_location(ls_obs)
    min_dist = float("inf")
    # FILL IN: COMPARE ls_read with ls_obs and find the best match
    for idx in range(signatures.size):
        print(
            "STATUS:  Comparing signature " + str(idx) + " with the observed signature."
        )
        ls_read = signatures.read(idx)
        his_read = convert_signature_to_hist(ls_read)
        his_obs = convert_signature_to_hist(ls_obs)
        dist = compare_historgrams(his_obs, his_read)
        if dist < min_dist:
            min_dist = dist
            best_idx = idx
    # return
    print("comparison completed, best index:", best_idx, "Best Distance:", min_dist)

    return best_idx, particles, ls_obs


def recognize_location2():
    """
    This function will compare the current signature to all signatures whcih are present and will determine accordingly which signature the location most accurately corresponds to.
    """
    ls_obs = LocationSignature(no_bins=bin_number)
    characterize_location(ls_obs)
    min_dist = float("inf")
    best_idx = 0
    # FILL IN: COMPARE ls_read with ls_obs and find the best match
    for idx in range(signatures.size):
        print(
            "STATUS:  Comparing signature " + str(idx) + " with the observed signature."
        )
        ls_read = signatures.read(idx)
        ls_read = signatures.read(idx)
        his_read = convert_signature_to_hist(ls_read)
        his_obs = convert_signature_to_hist(ls_obs)
        dist = compare_historgrams(his_obs, his_read)
        if dist < min_dist:
            min_dist = dist
            best_idx = idx

    # return
    print("comparison completed, best index:", best_idx, "Best Distance:", min_dist)

    return best_idx


# Prior to starting learning the locations, it should delete files from previous
# learning either manually or by calling signatures.delete_loc_files().
# Then, either learn a location, until all the locations are learned, or try to
# recognize one of them, if locations have already been learned

# signatures = SignatureContainer(5)
# learn_location()
