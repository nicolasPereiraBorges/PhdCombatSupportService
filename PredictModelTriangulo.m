function [scores, v, f] = PredictModelTriangulo(model, medianas, pesos)
% Extrai Verdadeiros e falsos
%d = model(model(:,2)>=0,:);
d = model;
% fraud = d(:,1);
% d = d(:,2:end);

fraud = d(:,end);
d = d(:,1:end-1);

%nTriangulos = size(d,2);
% Verifica qtd de pesos;
nMedianas = size(medianas,1);
scores = zeros(nMedianas, size(d,1));
for i = 1: nMedianas
    linha = medianas(i,:);     
    minimos = min(linha,d);
    maximos = max(linha,d);    
    div = (minimos./maximos);    
    s_aux = div.* pesos(i,:);
    scores(i,:) = sum(s_aux,2)';
    %scores(i,:) = sum(((minimos./maximos).* pesos(i,:))');
end
if (size(scores,1) > 1)
    scores = [fraud, max(scores)'];
else
    scores = [fraud, scores'];
end
%

v = (scores(scores(:,1)==0,2));
f = (scores(scores(:,1)==1,2));
end

    







