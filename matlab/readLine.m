function line = readLine(fid)
    while true
        line = fgetl(fid);
        if ~isempty(line)
            break;
        end
    end
end
