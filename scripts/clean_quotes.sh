#!/bin/bash



# move quoters behind comments
sed 's/!/./'      100_inspirational_quotes.txt > temp_quotes.txt
sed -i -e 's/\..*~/\"\}\, \/\/ /' temp_quotes.txt

# clean empty lines
sed -i -r '/^\s*$/d' temp_quotes.txt

# add quotes
sed -i -e 's/^/   \{\"/' temp_quotes.txt


echo 'char *quotes[] = {' >   inspirational_quotes.c
cat  temp_quotes.txt >>       inspirational_quotes.c
echo '};' >>                  inspirational_quotes.c
rm  temp_quotes.txt
