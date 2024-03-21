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


#Class to Send E-Mail:
class mail_smtp():
    def __init__(self):
        #Email Credentials:
        self.SMTP_User = 'dhinesh.r2401@gmail.com'
        self.SMTP_Pass = 'ftxjlqcabkoxpdyv'

        #Email Destination Details:
        # self.email_to = 'ENPM809TS19@gmail.com'
        self.email_to = 'dhineshrajasekaran@gmail.com'
        self.email_from = self.SMTP_User
        # self.email_cc = 'rpatil10@umd.edu'
        self.email_cc = 'dhinesh@umd.edu'
        self.email_check = 'dhineshrajasekaran@gmail.com'


    #Function to Send E-Mail:
        #Input: Email Subject, Email Content
        #Ouput: Sends Email and Prints Status
    def mail_snapshot(self, subject_email, content_email):
        try:
            #Define time stamp & record an image:
            pic_time = datetime.now().strftime("%Y%m%d%H%M%S")
            i = 1
            command = 'raspistill -w 640 -h 480 -o pic' + str(i) + '.jpg' #Takes a picture
            os.system(command)

            #Email Subject/From/To:
            msg = MIMEMultipart()
            msg['Subject'] = subject_email 
            msg['From'] = self.email_from
            msg['To'] = self.email_to

            #Email Text/Body:
            body = MIMEText(content_email)
            msg.attach(body)
                
            #Attach Image to Email:
            fp = open('pic'+str(i)+'.jpg', 'rb')
            img = MIMEImage(fp.read())
            fp.close()
            msg.attach(img)

            #Send email
            s = smtplib.SMTP('smtp.gmail.com', 587)

            s.ehlo()
            s.starttls()
            s.ehlo()

            print("Sending Email...")
            s.login(self.SMTP_User, self.SMTP_Pass)
            s.sendmail(self.email_from, [self.email_to, self.email_cc, self.email_check], msg.as_string())
            s.quit()
            print("Email Delivered")


        except Exception as e:
            print(e)

    

def main():
    p1 = mail_smtp()
    p1.mail_snapshot('809T Robot Status - dhinesh - 119400241', 'Object Picked')


if __name__ == "__main__":
    main()