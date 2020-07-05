import multiprocessing
from numpy import random

# Metodo da parallelizzare.
# index serve per sapere "dove sono" e salvare il result all'interno della cella del dict giusta.
# ad esempio se lancio la ricerca numero 1 con tot scatole salvo il risultato nel dict alla posizione 1.
# se poi randomizzi un po le scatole rifarai una chiamata ma con index 2 per distinguere i due casi ecc...
# Quindi insomma risultato e quello che metteresti nel return, e index serve per scrivere nnellla posizione giusta del risultato.
def Somma(x, y, index, risultato):
    print('Sommo')

    res = 0
    for i in range(len(x)):
        res += x[i] + y[i]

    print('Ho sommmato')

    risultato[index] = res


if __name__ == '__main__':

    manager = multiprocessing.Manager()

    return_values = manager.dict() #  CREO LA STRUTTURA DATI CONDIVISA TRA TUTTI I PRCCESSI
    jobs = [] # LISTA DI PROCESSI CHE LANCIO
    NUM_PROCESSES = 1000

    for index in range(NUM_PROCESSES):
        # target e' la funzione da chiamare nel processo, args sono i suoi argomenti in ordine.
        p = multiprocessing.Process(target=Somma, args=(random.rand(10).tolist(), random.rand(10).tolist(), index, return_values))
        jobs.append(p)
        p.start() # lancio l'effettiva esecuzione del processo

    # al termine del for ho lanciato tutti i NUM_PROCESS che volevo lanciare.
    # chiamo la join su ognuno di questi processi cosi sono sicuro che abbiano finito.

    # Aspetto finche' tutti i processi in jobs siano finiti
    for process in jobs:
        process.join()

    # se sono qui ho finito tutte le join, ovvero tutti i processi chiamati sono terminati. Ora posso analizzare la struttura dati
    # condivisa e farci quello che voglio. essendo un dict ci faccio un looppettino cosi a botto ma e' solo una idea.
    # Appena sono tutti finiti valuto i risultati
    print('Analisi dei risultatiiiii: \n')
    for result in return_values.keys():
        print(return_values.values()[result])

    print('finitoooo')