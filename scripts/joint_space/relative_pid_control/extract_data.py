import os
import numpy as np


def extract_data(process_funtion, output_file_name=None,
                 root="Data", file_suffix="csv"):
    """ Extract data from files and save to a file using given function

    Process function should take input (data, p_i, t_i)
    """
    # Task information
    task_names = ["Task1", "Task2", "Task3"]
    headers = ",".join(task_names)

    # Each participant has one data folder
    if (not os.path.exists(root)):
        print("Root does not exist.")
    # Get data folder of each participant
    dir_path = next(os.walk(root))
    participant_folders = dir_path[1]

    # Container to store all the results
    # ROW: participants, COL: tasks
    values = np.zeros((len(participant_folders), len(task_names)))
    values = values.astype(object)

    # Process each file
    for p_i, participant in enumerate(participant_folders):
        # Get data file of each given task
        data_folder_path = root + "/" + participant
        files = os.listdir(data_folder_path)

        for f_i, file in enumerate(files):
            # task name
            task_name = file.split("."+file_suffix)[0]
            if (task_name not in task_names):
                continue
            t_i = task_names.index(task_name)

            # Load data
            file_path = data_folder_path + "/" + file
            data = np.loadtxt(open(file_path, "rb"), delimiter=",")
            # Process data
            value = process_funtion(data, p_i, t_i)
            # Record value
            values[p_i, t_i] = str(value)

    # Store the final result
    if (output_file_name is not None):
        np.savetxt(output_file_name+".csv", values, delimiter=",", fmt="%s",
                   header=headers, comments='')


def extract_completion_time(data, p_i, t_i):
    ''' Compute the time taken to complete the task
        based on the gaze data length
    '''
    data = np.atleast_2d(data)

    # Related column
    indices = [0, 1]
    # Process
    time = data[:, indices[0]:indices[1]]
    completion_time = time[-1, 0] - time[0, 0]
    return completion_time


def main():
    # Data path
    root = os.path.abspath("Data")
    file_type = "csv"
    # Output path
    output_root = os.path.abspath("Output")
    if (not os.path.exists(output_root)):
        os.mkdir(output_root)

    # General performance
    extract_data(extract_completion_time, output_root+"/time", root, file_type)


if __name__ == "__main__":
    main()
