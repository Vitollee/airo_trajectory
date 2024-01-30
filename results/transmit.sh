
cd ~/airo_trajectory_ws/src/airo_trajectory/results
zip -r datasave.zip *.csv
mv datasave.zip ~/datasave.zip
cd ~
sudo scp -p 22 datasave.zip patrick@127.0.0.1:~/LiFanyi
rm datasave.zip