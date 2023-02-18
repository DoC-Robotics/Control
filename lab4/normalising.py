# Particles_set from ParticlesMCL class
def sum_of_new_weights(particles_set):

    total_weight = 0

    for i in range(len(particles_set)):
        total_weight += particles_set.weights[i]

    return total_weight


def normalise_weights(particles_set, total_weight):

    for i in range(len(particles_set)):
        particles_set.weights[i] = particles_set.weights[i] / total_weight
    
    