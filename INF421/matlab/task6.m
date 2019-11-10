fidin = fopen('./inputs/456input.txt', 'r');
fidout = fopen('./outputs/6matlab_output.txt', 'w');

string1 = readLine(fidin);
while ischar(string1)
    string2 = readLine(fidin);
    [score, alignment] = swalign(string1,string2, 'ScoringMatrix', blosum(50), 'GapOpen', 8, 'ExtendGap', 1);
    fprintf(fidout, 'Score = %d\n', round(score));
    fprintf(fidout, [alignment(1,:), '\n']);
    fprintf(fidout, [alignment(3,:), '\n\n']);
    string1 = readLine(fidin);
end
fclose(fidin);
fclose(fidout);