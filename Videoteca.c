#include<stdio.h>
#include<string.h>
#define N_FILM 50

typedef struct{
    char nome[30];
    char regista[20];
    int anno;
}single_film;

struct videoteca{
    single_film film[N_FILM];
    int n;
};

void insert_films(struct videoteca *videoteca1)
{

    int n_film=0;
    do {
        printf("Inserisci numero di film da inserire  ( MASSIMO 50)\n");
        scanf("%d", &n_film);
    }while(n_film <= 0 || n_film > 50);
    videoteca1->n = n_film;
    for(size_t i =0; i < n_film; i++)
    {
        printf("Inserisci nome film\n");
        scanf("%*c%[^\n]",&((*videoteca1).film[i].nome));
        printf("Inserisci regista film\n");
        scanf("%*c%[^\n]",&((*videoteca1).film[i].regista));
        printf("Inserisci anno film\n");
        scanf("%d",&((*videoteca1).film[i].anno));
    }
    for(size_t i =0; i < n_film; i++)
    {
        printf("Titolo : \t%s\n",(*videoteca1).film[i].nome);
        printf("Regista : \t%s\n",(*videoteca1).film[i].regista);
        printf("Anno : \t%d\n",(*videoteca1).film[i].anno);
    }
}

int main()
{
    char regista_richiesto[20];
    struct videoteca videoteca1;
    single_film film_regista = {{0},{0},0};

    insert_films(&videoteca1);

    printf("Inserisci il regista di cui vuoi trovare il film più recente\n");
    scanf("%*c%19[^\n]%*c",regista_richiesto);

    for(size_t i;i < videoteca1.n;i++)
    {
        if(!strcmp(videoteca1.film[i].regista,regista_richiesto)){
            if(videoteca1.film[i].anno > film_regista.anno)
            {
                film_regista = videoteca1.film[i];
            }
        }
    }

    printf("Il film piu recente di %s e'\n\n",regista_richiesto);
    printf("Titolo : \t%s\n",film_regista.nome);
    printf("Regista : \t%s\n",film_regista.regista);
    printf("Anno : \t%d\n",film_regista.anno);
    return 0;
}
