import numpy as np
import os
import pdb
import matplotlib.pyplot as plt
import scipy.stats as stats
import math

cwd = os.getcwd()  # Get the current working directory (cwd)
files = os.listdir(cwd)  # Get all the files in that directory
# print("Files in %r: %s" % (cwd, files))
# print("Current working directory: ", cwd)

def read_txtfile_data(pathstr):
    ms_times_array = np.array([])
    with open(pathstr, 'r') as file:
        # Iterate through each line in the file
        for line in file:
            # Split the line into its components
            components = line.split()
            # Extract the millisecond time from the second component
            ms_time = float(components[7])
            # Append the millisecond time to the NumPy array
            ms_times_array = np.append(ms_times_array, ms_time)
    
    return ms_times_array

def gaussian_dist_params(data_arr):
    return [np.mean(data_arr), np.std(data_arr), np.var(data_arr)]

def plot_gaussian(data_arr, delay_time):

    mu, sigma, _ = gaussian_dist_params(data_arr)
    count, bins, ignored = plt.hist(data_arr, 30, density=True)
    plt.plot(bins, 1/(sigma * np.sqrt(2 * np.pi)) * np.exp( - (bins - mu)**2 / (2 * sigma**2)), linewidth=2, color='r')
    # plt.plot(data_arr, stats.norm.pdf(data_arr, mu, sigma))
    plt.xlabel("time, ms")
    plt.ylabel("density of probability")
    # plt.text(95, 0.03, 'Communication time delay in ms: {}'.format(delay_time), style='italic',
    #     bbox={'facecolor': 'none', 'alpha': 0.3, 'pad': 8})

    # plt.annotate('Communication delay in ms: {}'.format(delay_time), xy=(2, 1), xytext=(3, 4),
    #         arrowprops=dict(facecolor='black', shrink=0.05))
    plt.title('Communication time delay in ms: {}'.format(delay_time))

    plt.show()


if __name__ == "__main__":

    notimedel_path = '/home/sgari/iral_research/multi_new_mmm/src/resource_aware_coordination/data/no_time_delay.txt'
    ms_50_del_path = '/home/sgari/iral_research/multi_new_mmm/src/resource_aware_coordination/data/50_ms_delay.txt'
    ms_500_del_path = '/home/sgari/iral_research/multi_new_mmm/src/resource_aware_coordination/data/500_ms_delay.txt'

    # np.array type data structures on which you could do statistical analysis
    notimedel_arr = read_txtfile_data(notimedel_path)
    ms_50_del_arr = read_txtfile_data(ms_50_del_path)
    ms_500_del_arr = read_txtfile_data(ms_500_del_path)

    # get mean and standard deviation for above arrays
    notimedel_arr_mean, notimedel_arr_std, _ = gaussian_dist_params(notimedel_arr)
    print("no time delay MEAN: ", notimedel_arr_mean, " no time delay STD DEV: ", notimedel_arr_std)

    ms_50_del_arr_mean, ms_50_del_arr_std, _ = gaussian_dist_params(ms_50_del_arr)
    print("50 ms delay MEAN: ", ms_50_del_arr_mean, " 50 ms delay STD DEV: ", ms_50_del_arr_std)

    ms_500_del_arr_mean, ms_500_del_arr_std, _ = gaussian_dist_params(ms_500_del_arr)
    print("500 ms delay MEAN: ", ms_500_del_arr_mean, " 500 ms delay STD DEV: ", ms_500_del_arr_std)

    # plotting data
    plot_gaussian(notimedel_arr, 0.)
    plot_gaussian(ms_50_del_arr, 50.)
    plot_gaussian(ms_500_del_arr, 500.)


