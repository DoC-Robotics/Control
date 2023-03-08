import random
import likelihood
import particlesMCL

"""" This file contains the functions for normalising and resampling the particles. """


# Particles_set from ParticlesMCL class
def sum_of_new_weights(particles_set):
    """This function returns the sum of the weights of the particles."""
    total_weight = 0

    for i in range(particlesMCL.NUMBER_OF_PARTICLES):
        total_weight += particles_set.weights[i]

    return total_weight


def normalise_weights(particles_set, total_weight):
    """ " This function normalises the weights of the particles."""

    for i in range(particlesMCL.NUMBER_OF_PARTICLES):
        particles_set.weights[i] = particles_set.weights[i] / total_weight
    return particles_set


def get_particles_cumulative_prob_dict(particle_set):
    """This function returns a dictionary with the cumulative probability of each particle."""

    cumulative_dictionary = {}

    for i in range(particlesMCL.NUMBER_OF_PARTICLES):
        if i == 0:
            cumulative_dictionary[i] = particle_set.weights[i]
        else:
            cumulative_dictionary[i] = (
                cumulative_dictionary[i - 1] + particle_set.weights[i]
            )

    return cumulative_dictionary


def get_particle_indexes_from_cumulative_prob_dict(particle_set, cumulative_dictionary):
    """ " This function returns a particle index based on the cumulative probability dictionary."""
    random_n = random.uniform(0, 1)
    for i in range(particlesMCL.NUMBER_OF_PARTICLES):
        if random_n <= cumulative_dictionary[i]:
            return i


def get_new_particle_index_set(particle_set, cumulative_dictionary):
    """This function returns an array of 100 new particle indexes."""
    new_particle_index_set = []

    # This generates 100 new particles indexes and appends to array
    for _ in range(particlesMCL.NUMBER_OF_PARTICLES):
        index = get_particle_indexes_from_cumulative_prob_dict(
            particle_set, cumulative_dictionary
        )
        new_particle_index_set.append(index)

    return new_particle_index_set


def update_old_particle_set(particle_set, new_particle_index_set):
    """ " This function updates the old particle set with the new particle set."""
    new_particle_set = particlesMCL.particlesMCL()

    # Copying new particle set coordinates and weighting
    for i in range(particlesMCL.NUMBER_OF_PARTICLES):
        new_particle_set.coordinates[i][0] = particle_set.coordinates[
            new_particle_index_set[i]
        ][0]
        new_particle_set.coordinates[i][1] = particle_set.coordinates[
            new_particle_index_set[i]
        ][1]
        new_particle_set.coordinates[i][2] = particle_set.coordinates[
            new_particle_index_set[i]
        ][2]
        new_particle_set.weights[i] = particle_set.weights[i]

    particle_set = new_particle_set
    return particle_set


def normalising_and_resampling(particles_set):
    """ " This function normalises the weights of the particles and resamples the particles."""
    # Normalising
    total_weights = sum_of_new_weights(particles_set)
    normalised_particles_set = normalise_weights(particles_set, total_weights)

    # Resampling
    cumulative_dict = get_particles_cumulative_prob_dict(normalised_particles_set)
    new_particle_index_set = get_new_particle_index_set(particles_set, cumulative_dict)
    new_particle_set = update_old_particle_set(particles_set, new_particle_index_set)
    return new_particle_set


def normalising_and_resampling2(particles_set):
    """ " This function normalises the weights of the particles and resamples the particles."""
    # Normalising
    total_weights = sum_of_new_weights(particles_set)
    normalised_particles_set = normalise_weights(particles_set, total_weights)

    # Resampling
    cumulative_dict = get_particles_cumulative_prob_dict(normalised_particles_set)
    new_particle_index_set = get_new_particle_index_set(particles_set, cumulative_dict)
    new_particle_set = update_old_particle_set(particles_set, new_particle_index_set)
    return new_particle_set


# TESTBENCH
"""
def test_rand_samp():   
    #initialize a dictionary with random values
    my_dict = {0:0.02,1:0.1,2:0.18,3:0.25,
    4:0.40,5:0.45,6:0.6,7:0.85,
    8:0.95,9:1.0}    
    
    #gets the array
    arr = get_new_particle_index_set(my_dict)

    print("get_new_particle_index_set: ", arr)

    return arr

"""


def test_normalising_and_resampling():
    None


if __name__ == "__main__":
    # test_rand_samp()
    particle_set = particlesMCL.particlesMCL()
    new_particle_set = normalising_and_resampling(particle_set)

    pass
