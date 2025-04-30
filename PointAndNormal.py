import File
import os


# THIS IS FOR SEPARATING POINT CLOUD AND NORMAL BECOME ONE FILE

FileCount = 0
d = './Layer'
for path in os.listdir(d):
    if os.path.isfile(os.path.join(d,path)):
        FileCount +=1

print('Total files in the folder (point file and normal file): ', FileCount)

if not os.path.exists('Output File/Part Layer Normal {}'):
    print('Note: If it is error, Make sure to close the "Output File" folder first and run it again.')
    os.makedirs('Output File/Part Layer Normal')

if FileCount > 2:
    for NoOfLayer in range(0, int(FileCount/2)):
        MixPoint = File.ReadXyzFile('Layer/Layer {}.xyz'.format(NoOfLayer))
        TopPoint = File.ReadXyzFile('Output File/Part Layer/Top_Layer_{}.xyz'.format(NoOfLayer))
        BotPoint = File.ReadXyzFile('Output File/Part Layer/Bot_Layer_{}.xyz'.format(NoOfLayer))

        MixNormalPoint = File.ReadXyzFile('Layer/Layer {} normal.xyz'.format(NoOfLayer))

        TopLayerNormal = []
        BotLayerNormal = []

        for NoOfPoint in range(0, len(MixPoint)):
            # print('\nPoint No:', NoOfPoint)
            PointAtTopDetected = False

            # Check point at Top Point File
            for NoOfTopPoint in range(0, len(TopPoint)):
                if MixPoint[NoOfPoint] == TopPoint[NoOfTopPoint]:
                    TopLayerNormal.append(MixNormalPoint[NoOfPoint])
                    PointAtTopDetected = True
                    # print('Ketemu di top')
                    break

            # Check point at Bot Point File if the point is not inside Top Point File
            if PointAtTopDetected == False:
                # print('After detected', PointAtTopDetected)
                for NoOfBotPoint in range(0, len(BotPoint)):
                    if MixPoint[NoOfPoint] == BotPoint[NoOfBotPoint]:
                        # print('Ketemu di Bot')
                        BotLayerNormal.append(MixNormalPoint[NoOfPoint])
                        break


        # print('Top Layer Normal:')
        # print(TopLayerNormal)
        File.SaveFile('Output File/Part Layer Normal/Top_Layer_Normal_{}'.format(NoOfLayer),TopLayerNormal)
        # print('Bot Layer Normal')
        # print(BotLayerNormal)
        File.SaveFile('Output File/Part Layer Normal/Bot_Layer_Normal_{}'.format(NoOfLayer), BotLayerNormal)


