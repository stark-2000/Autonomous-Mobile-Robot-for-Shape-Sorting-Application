import qrcode
print("QR Package imported successfully")

code = qrcode.make("ENPM809T")
print("QR Code generated successfully !")

code.save("generated_qr.png")
print("QR Code saved !")


