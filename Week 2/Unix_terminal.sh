mkdir ~/logs
sudo mv /root/*log* ~/logs/

#'mkdir' to create a '/logs' folder in the home directory 
#'sudo' to authorize the permission to non-root user to modify the files contained in /root folder
#'mv' to move the target log files 'root/' to the destination '~/logs/'
#'*log*' is used as wildcard pattern, to match any filename containing 'log' substring