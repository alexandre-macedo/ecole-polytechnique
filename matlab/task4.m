fidin = fopen('in', 'r');
fidout = fopen('out', 'w');

string1 = readLine(fidin);
while ischar(string1)
    string2 = readLine(fidin);
    [score, alignment] = nwalign(string1,string2, 'ScoringMatrix', blosum(50), 'GapOpen', 0);
    fprintf(fidout, 'Score = %d\n', round(score));
    fprintf(fidout, [alignment(1,:), '\n']);
    fprintf(fidout, [alignment(3,:), '\n\n']);
    string1 = readLine(fidin);
end
fclose(fidin);
fclose(fidout);