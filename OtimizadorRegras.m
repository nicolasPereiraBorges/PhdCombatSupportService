function [melhorI,melhorJ,med,pes,desv,result, melhorV, melhorF]= OtimizadorRegras(t)
melhor = -1;
melhorI = -1;
melhorJ = -1;
med = [];
pes = [];
desv = [];
for jkk = 1: 15
fprintf('Processing %d \n', jkk);
for nGrupos = 20: 35
[medianas, pesos, desvios_aux] =  AvaliaPesosModel(t(t(:,end)==1, 1:end-1), nGrupos);
[scores, f, v] = PredictModelTriangulo(t, medianas, pesos);
for i = 98:-0.01:90
odds = ((sum(f<=i)/length(f))*100) / ((sum(v<=i)/length(v))*100);
 %fprintf('Odds - %.2f\n', odds);
if (odds < inf) &&(odds > melhor) && (sum(f<=i)/length(f))*100 > 60
%if (odds < inf) &&(odds > melhor) && (sum(v<=i)/length(v))*100 < 0.4 && (sum(f<=i)/length(f))*100 > 40
melhor = odds;
melhorI = nGrupos;
melhorJ = i;
med = medianas;
pes = pesos;
desv = desvios_aux;
melhorV = v;
melhorF = f;
melhorScore = scores;
end
end
fprintf('Melhor  Odds = %.2f -- Ngrupos =%d\n', melhor, melhorI);
end
end

result =[];
for i = 98:-0.01:75
fp = (sum(melhorV<=i)/length(melhorV))*100;
vn = (sum(melhorF<=i)/length(melhorF))*100;
result = [result;i,vn,fp, vn./fp];
end
result



%[scoresA, fA, vA] = PredictModelTriangulo(tA, med, pes);

