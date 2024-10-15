#!/bin/bash
#SBATCH --job-name=PSI_stereo            # job name
#SBATCH --nodes=1
#SBATCH --ntasks=1                  # number of tasks across all nodes
#SBATCH --cpus-per-task=16
#SBATCH --partition=general          # name of partition to submit job
#SBATCH --time=24:00:00              # Run time (D-HH:MM:SS)
#SBATCH --output=job-%j.out          # Output file. %j is replaced with job ID
#SBATCH --error=job-%j.err           # Error file. %j is replaced with job ID
#SBATCH --mail-type=ALL              # will send email for begin,end,fail
#SBATCH --mail-user=dcs0008@auburn.edu

module load matlab 
matlab -nodisplay -nosplash -r disparity_calculation >> disparity_output.txt
