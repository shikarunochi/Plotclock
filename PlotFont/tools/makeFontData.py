import sys
import binascii

args = sys.argv
 
fileName = args[1]
#fileName="KST32B.TXT"
outFileName = args[2]
#outFileName = "out.txt"
fontFile = open(fileName, 'r',encoding='shift_jis')
outputFile = open(outFileName, 'w',encoding='utf-8')

fontDataIndex='const int *fontDataIndex[]={\n'
fontCodeIndex='const uint16_t fontIndex[]={\n'
fontDataList =""

for line in fontFile:
    if line.startswith('*'):
        continue
    data = line.split(' ')
    if len(data) < 2:
        continue
    fontJISCode = data[0]
    try:
        if int(fontJISCode, 16) < 256:
            charactor = chr(int(fontJISCode, 16))
        else:
        # < 0x5000 
            if int(fontJISCode, 16) < 0x5000:
                charactor = binascii.unhexlify("1b2442" + fontJISCode).decode('iso-2022-jp')
            else:
                continue
    except:
        charactor=""

    fontDataList = fontDataList + "const int font" + fontJISCode + "[] = {"
    fontData = data[1]
    firstFlag = True
    for c in fontData:
        if firstFlag == False:
            fontDataList = fontDataList + ','
        else:
            firstFlag = False
        fontDataList = fontDataList + hex(int.from_bytes(c.encode("shift_jis"),'little'))
    fontDataList = fontDataList +  ',0x20};//[' + charactor + ']\n'
    fontDataIndex = fontDataIndex + "  font" + fontJISCode + ',\n'
    if len(charactor) > 0:
        fontCodeIndex = fontCodeIndex + "  0x" +  charactor.encode(encoding='utf-16be').hex() +',//[' + charactor + ']\n'
    else:
        fontCodeIndex = fontCodeIndex + "  0" + ',\n'

fontDataIndex = fontDataIndex + '};\n'
fontCodeIndex = fontCodeIndex + '};\n'
outputFile.write(fontDataList)
outputFile.write('\n')
outputFile.write(fontDataIndex)
outputFile.write('\n')
outputFile.write(fontCodeIndex)

outputFile.close()
fontFile.close()