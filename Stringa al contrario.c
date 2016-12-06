#include<stdio.h>
#include<string.h>

int main()
{
    char frase_letta[21];
    int dim = 0;
    printf("Inserisci una frase (massimo 20 caratteri)\n");
    scanf("%[^\n]s",frase_letta);
    dim = strlen(frase_letta);
    printf("La frase al contrario e' ");
    for (int add = dim; add >= 0; add--)
    {
        printf("%c",frase_letta[add]);
    }
    printf("\n");
    return 0;
}
