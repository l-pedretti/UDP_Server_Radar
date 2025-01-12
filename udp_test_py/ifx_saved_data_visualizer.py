import os
import pickle
import matplotlib.pyplot as plt

# Specify the folder path
folder_path = 'c:\\Users\\pedro\\saved_data\\25_11_2024-15_05_06'

# Create a figure and axis
fig, ax = plt.subplots()

# Loop through all files in the folder
for filename in os.listdir(folder_path):
    # Check if the file is a pickle file
    if filename.endswith('.pkl'):
        # Load the pickle file
        with open(os.path.join(folder_path, filename), 'rb') as f:
            data = pickle.load(f)
        
        # Visualize the data
        data = data / data.max()
        ax.imshow(data.reshape(64,32,3))
        #plt.imshow(image, cmap='gray')
        ax.set_title(filename)
        plt.draw()
        plt.pause(0.1)  # Add a 100ms delay
        ax.cla()  # Clear the axis for the next plot
plt.show()