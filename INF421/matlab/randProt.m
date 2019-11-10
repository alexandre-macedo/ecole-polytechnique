Alphabet = ['A', 'R', 'N', 'D', 'C', 'Q', 'E', 'G', 'H', 'I', 'L', 'K', 'M', 'F', 'P', 'S', 'T', 'W', 'Y', 'V'];
Size = floor(rand*20) +1000;
Protein = '';
for i = 1:Size
    index = floor(rand*20)+1;
    Protein = strcat(Protein, Alphabet(index));
end

Protein
