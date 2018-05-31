echo -e "\033[32m Please select region\033[0m"
echo -e "\033[32m   1:AU\033[0m"
echo -e "\033[32m   2:CN470\033[0m"
echo -e "\033[32m   3:EU\033[0m"
echo -e "\033[32m   4:JP\033[0m"
echo -e "\033[32m   5:TW\033[0m"
echo -e "\033[32m   6:US\033[0m"

read -p "" REGION
echo "loraNodeLib/" > .mbedignore
rm lib*.a

if [ "$REGION" == "1" ]; then
  echo "Ok, build AU"
  cp loraNodeLib/*AU*.a .
elif [ "$REGION" == "2" ]; then
  echo "Ok, build CN470"
  cp loraNodeLib/*CN470*.a .
elif [ "$REGION" == "3" ]; then
  echo "Ok, build EU"
  cp loraNodeLib/*EU*.a .
elif [ "$REGION" == "4" ]; then
  echo "Ok, build JP"
  cp loraNodeLib/*JP*.a .
elif [ "$REGION" == "5" ]; then
  echo "Ok, build TW"
  cp loraNodeLib/*TW*.a .
elif [ "$REGION" == "6" ]; then
  echo "Ok, build US"
  cp loraNodeLib/*US*.a .
else
	echo "illegal input $REGION"
	exit 0
fi

mbed compile -t GCC_ARM -m MTB_ADV_WISE_1510  -c
