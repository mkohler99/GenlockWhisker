# Genlock Whisker
<img width="846" height="538" alt="Screenshot 2025-11-01 at 7 53 55 AM" src="https://github.com/user-attachments/assets/c5de6d43-dbd9-4a36-a791-0f2e946aa108" />
<img width="528" height="820" alt="Screenshot 2025-11-01 at 7 54 20 AM" src="https://github.com/user-attachments/assets/a2ef13ae-6b82-4c85-815a-56dafac129db" />

#Exciting Updates!
Now with a web serial UI. Simply host the contents of the 'web' folder somewhere and open the page with Chrome (or maybe Edge? Safari does not allow web serial which makes me sad because web serial is pretty awesome) . Im gonna host this somewhere eventually as an example. TBD I *think* this requires HTTPS and you cant just open the HTML on your local machine but id love to be wrong. Anyway, the Web UI adds easier to read monitoring as well as an attempt at monitoring the statistics over time. Im trying to verify how useful ANY of these jitter measurments are and would love if anyone has......a bad genlock source that has lots of jitter to test with?
<img width="1088" height="1045" alt="Screenshot 2025-12-03 at 4 25 40 PM" src="https://github.com/user-attachments/assets/80fa4037-8241-41e2-9f53-463d79424b9c" />

# About
The genlock whisker is an expansion module for the FUSE CATS Frame:Work 2025 NYC Pixel Kitty conference badge that allows you to turn your badge into a genlock tester. 
The device reads most types of genlock signals on an HDBNC connector and displays information about them on the badge's RGB Pixel Matrix Display.

# Current features:
- Detects Bi-Level or Tri-Level sync by displaying green for Bi-Level and blue for Tri-Level. Detects no sync or unstable sync by displaying "NO LOCK" in red
- Displays Frame rate of sync up to 3 decimal places
- Outputs sync drift statistics over USB type C serial connection

- Hardware fix for This latest code release.
<img width="1187" height="786" alt="Screenshot 2025-12-03 at 4 10 32 PM" src="https://github.com/user-attachments/assets/bb182807-2e30-4100-9e80-eeda05806dc5" />
Cut the indicated Pin in order to un-short the HSYNC IO Pin. Since this is actually a bug in the base Pixel Kitty Badge Board, its hard to fix perfectly since the problem can be in either place. for now, the easiest fix is to remove one header pin from the expansion module which prevents the HSYNC pin on the LMH1980 from grounding out and not being able to read. because this new version improves data gathering by detecting HSYNC lines and counting them in order to determine the signal format, this version of software will not work without the hardware mod. <img width="504" height="584" alt="Screenshot 2025-12-03 at 4 22 02 PM" src="https://github.com/user-attachments/assets/c8c71b5d-231c-4fa5-9ba2-2851c770d75b" />

Because Safety is our number 1 priority, please wear safety goggles when cutting the pin as it can fly anywhere.


