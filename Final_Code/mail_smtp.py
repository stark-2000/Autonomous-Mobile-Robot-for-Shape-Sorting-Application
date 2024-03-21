#Dependencies:
from email.mime.multipart import MIMEMultipart
from email.mime.image import MIMEImage
from email.mime.text import MIMEText
from smtplib import SMTPException
from datetime import datetime
from smtplib import SMTP
import smtplib
import email
import os

from picamera import PiCamera
# import cv2


#Class to Send E-Mail:
class mail_smtp():
    def __init__(self):
        #Initialize the Raspberry Pi camera
        # self.camera = PiCamera()
        # self.camera.resolution = (640, 480)
        # self.camera.framerate = 25
        # sleep(0.1) #Allow the camera to warmup

        #Email Credentials:
        self.SMTP_User = 'dhinesh.r2401@gmail.com'
        self.SMTP_Pass = 'ftxjlqcabkoxpdyv'

        #Email Destination Details:
        self.email_to = 'ENPM809TS19@gmail.com'
        # self.email_to = 'dhineshrajasekaran@gmail.com'
        self.email_from = self.SMTP_User
        self.email_cc = 'rpatil10@umd.edu'
        # self.email_cc = 'dhinesh@umd.edu'
        # self.email_cc = 'arun45000@gmail.com'

        #Send email
        self.s = smtplib.SMTP('smtp.gmail.com', 587)
        self.s.ehlo()
        self.s.starttls()
        self.s.ehlo()
        self.s.login(self.SMTP_User, self.SMTP_Pass)


    #Function to Send E-Mail:
        #Input: Email Subject, Email Content
        #Ouput: Sends Email and Prints Status
    def mail_snapshot(self, subject_email, content_email, camera):
        try:
            #Define time stamp & record an image:
            # pic_time = datetime.now().strftime("%Y%m%d%H%M%S")
            i = 1
            camera.capture('img'+str(i)+'.png', use_video_port=False)
            # image1 = MIMEImage(cv2.imread('img'+str(i)+'.png'))
            
            # command = 'raspistill -w 640 -h 480 -o pic' + str(i) + '.jpg' #Takes a picture
            # os.system(command)

            #Email Subject/From/To:
            msg = MIMEMultipart()
            msg['Subject'] = subject_email 
            msg['From'] = self.email_from
            msg['To'] = self.email_to

            #Email Text/Body:
            body = MIMEText(content_email)
            msg.attach(body)
                
            #Attach Image to Email:
            fp = open('img'+str(i)+'.png', 'rb')
            image1 = MIMEImage(fp.read())
            fp.close()
            msg.attach(image1)

            print("Sending Email...")
            self.s.sendmail(self.email_from, [self.email_to, self.email_cc], msg.as_string())
            print("Email Delivered")


        except Exception as e:
            print(e)

    
    def close_email(self):
        self.s.close

# def main():
#     p1 = mail_smtp()
#     # p1.mail_snapshot('809T Robot Status - dhinesh - 119400241', 'Object Picked', camera)


# if __name__ == "__main__":
#     main()