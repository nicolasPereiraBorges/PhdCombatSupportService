function [medianas, pesos, desvios_aux] =  AvaliaPesosModel(model, nGrupos)
% Extrai Verdadeiros e falsos
%v = model(model(:,1)==0,1:end);
%v = v(v(:,1)>=0,1:end);
v = model;
nTriangulos = size(v,2);
%f = modelB(model(:,1)==1,2:end);
%f = f(f(:,1)>=0,:);
% Identifica padrao de verdadeiros
labels = kmeans(v, nGrupos,'dist','sqeuclidean');
medianas = zeros(nGrupos,nTriangulos);
desvios = zeros(nGrupos,nTriangulos);
desvios_aux = zeros(nGrupos,nTriangulos);
pesos = zeros(nGrupos,nTriangulos);
pesos_aux = zeros(nGrupos,nTriangulos);
% Computa medianas e std
for i = 1: nGrupos
    aux = v(labels==i,:);        
    if (size(aux,1) > 1)
        medianas(i,:) = mean(aux);    
        desvios(i,:) = std(aux);    
        desvios_aux(i,:) = std(aux);    
        pesos_aux = sort((desvios(i,:)./ sum((desvios(i,:)))).*100, 'ascend');    
        for j = 1: nTriangulos
           [~,c] = find(desvios(i,:) == max(desvios(i,:)));             
           c = c(1);       
           desvios(i,c) = -1;
           pesos(i,c) = pesos_aux(j);
        end    
    else
         medianas(i,:) = aux;    
         desvios(i,:) =  aux.*0;  
         pesos(i,:) =  100./length(aux);  
    end
end

for i = 1: nGrupos
    m = medianas(i,:);
    [~,c] = find(m==-1);
    if ~isempty(c)
        [~,c2] = find(m>=0);
        s =  sum(pesos(i,c));
        pesos(i,c) = 0;
        nMenos1 = length(c2);
        fator = (s/nMenos1);
        pesos(i, c2) = pesos(i, c2) + fator;
    end
end

%

% 
% smedianas = "List<float> medianas = new List<float>() {";
% for i = 1: size(medianas,1)
%     for j = 1: size(medianas,2)
%         smedianas = sprintf('%s%ff,', smedianas, medianas(i,j));
%     end
% end
% smedianas = smedianas(1:end-1) + "};";


% spesos = "List<float> pesos = new List<float>() {";
% for i = 1: size(pesos,1)
%     for j = 1: size(pesos,2)
%         spesos = sprintf('%s%ff,', spesos, pesos(i,j));
%     end
% end
% spesos = spesos(1:end-1) + "};";
% 
% fprintf('%s\n%s\n', smedianas,spesos);

end
    







