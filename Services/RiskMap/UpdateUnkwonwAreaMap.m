function riskMap = UpdateUnkwonwAreaMap(riskMap)
%CREATECOSTMAP Summary of this function goes here
%   Detailed explanation goes here
    return;
    safe = riskMap.Safe;
    noFly = riskMap.NoFly;
    checked = repmat(riskMap.CoveredArea,1).*0;
    checked(safe~=0) = inf;
    checked(noFly~=0) = inf;    
    %checked(riskMap.CoveredArea > 90) = inf;
    %checked(riskMap.CoveredArea < -10) = inf;
    %checked(noFly > 0) = noFly(noFly>0);
    %checked(checked<0) = 0;
        
    while (1)
        [maxI, maxJ] = GetMaxValueNoChecked(checked);
        if maxI < 1
            break;
        end
        checked(maxI, maxJ) = inf;
        neighbours = UncheckedNeigbours(riskMap, maxI, maxJ, checked);
        if ~isempty(neighbours)
            bestIndex = 1;            
            for i = 1: size(neighbours,1)
                [aux_sumSafe, aux_sumNoFly, aux_sumCovered, sumMap, aux_nNeighbours] = AnalyseNeihbours(riskMap, neighbours(i,1), neighbours(i,2));
                  value = 0.9.*(sumMap);%./(aux_nNeighbours);    
%                   if value == -50
%                      value = -30;
%                   end
                  if riskMap.CoveredArea(neighbours(i,1), neighbours(i, 2)) > -0.2 ...
                      && riskMap.CoveredArea(neighbours(i,1), neighbours(i, 2)) < 0.2
                      riskMap.CoveredArea(neighbours(i,1), neighbours(i, 2)) = value;                      
                      riskMap.Map(neighbours(i,1), neighbours(i, 2)) = value + ...
                          safe(neighbours(i,1), neighbours(i, 2)) + ...
                          noFly(neighbours(i,1), neighbours(i, 2));
                  end
                  %checked(neighbours(i,1), neighbours(i, 2)) = inf;
                  
                      
                  
                  %end
            end           
            %value = ((sumSafe - (sumNoFly) + sumCovered)./nNeighbours)/3;            
            
%              if (value > 0.5)
%                  value = 0.5;
%              end
%             if (value < 1)
%                 value = -0.5;
%             end
            
            
        end
        
        
    end       
    
end


function [maxI, maxJ] = GetMaxValueNoChecked(checked)

    maximo = min(min(checked));
    if (maximo < inf)
        [l,c] = find(checked == maximo);
        r = randi([1,length(l)]);
        maxI = l(r);
        maxJ = c(r);
    else        
        maxI = -1;
        maxJ = -1;
    end          
end

function [sumSafe, sumNoFly, sumCovered, sumMap, nNeighbours] = AnalyseNeihbours(riskMap, i, j)
    
    map = riskMap.Map;
    m = size(map,1);
    n = size(map,2);    
    
    if (j > 1)
        if j < n            
            j0 = j-1;
            j1 = j+1;
        else
            % ultima coluna            
            j0 = j-1;
            j1 = j;            
        end
    else
        % Primeira coluna        
        j0 = j;
        j1 = j+1;            
    end
    
    if (i > 1)
        if i < m            
            i0 = i-1;
            i1 = i+1;
        else
            % ultima linha            
            i0 = i-1;
            i1 = i;            
        end
    else
        % Primeira linha        
        i0 = i;
        i1 = i+1;            
    end
    
    ii = i0:i1;
    jj = j0:j1;
    
    sumSafe = sum(sum(riskMap.Safe(ii,jj)));
    sumNoFly = sum(sum(riskMap.NoFly(ii,jj)));
    sumCovered = sum(sum(riskMap.CoveredArea(ii,jj)));
    sumMap = sum(sum(riskMap.Map(ii,jj))) - riskMap.Map(i,j);
    sumMap = mean(mean(riskMap.Map(ii,jj)));
    nNeighbours = size(riskMap.Map(ii,jj),1) * size(riskMap.Map(ii,jj));
end


function [neighbours] = UncheckedNeigbours(riskMap, i, j, checked)
    
    map = riskMap.Map;
    m = size(map,1);
    n = size(map,2);    
    
    if (j > 1)
        if j < n            
            j0 = j-1;
            j1 = j+1;
        else
            % ultima coluna            
            j0 = j-1;
            j1 = j;            
        end
    else
        % Primeira coluna        
        j0 = j;
        j1 = j+1;            
    end
    
    if (i > 1)
        if i < m            
            i0 = i-1;
            i1 = i+1;
        else
            % ultima linha            
            i0 = i-1;
            i1 = i;            
        end
    else
        % Primeira linha        
        i0 = i;
        i1 = i+1;            
    end
    
    ii = i0:i1;
    jj = j0:j1;
        
    %[l,c] = find(checked(ii,jj) < 1 & checked(ii,jj) > -inf & riskMap.NoFly(ii,jj) < 1 & riskMap.Safe(ii,jj) < 1);
    [l,c] = find(checked(ii,jj) < inf);
    %[l,c] = find(riskMap.Map(ii,jj) < 100 | (checked(ii,jj) < 1 & checked(ii,jj) > -inf & riskMap.NoFly(ii,jj) < 1) );
    %[l,c] = find(riskMap.CoveredArea(ii,jj) == 0 | (checked(ii,jj) < 1 & checked(ii,jj) > -inf & riskMap.NoFly(ii,jj) < 1) );
    
    if (~isempty(l))
        l = l+i0-1;
        c = c+j0-1;
    end
    neighbours = [l,c];
end