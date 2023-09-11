
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <stdio.h> 
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>

int main(int argc, char **argv)
{
    
    int socket_serv, err;
    struct sockaddr_in addr;
    socket_serv = socket(AF_INET,SOCK_STREAM,0);
    if (socket_serv<0) { perror("a"); exit(1); }
    addr.sin_family = AF_INET;
    addr.sin_port = htons(9000);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    err = bind(socket_serv, (struct sockaddr *) &addr, sizeof(struct sockaddr_in));
    if (err<0) { perror("b"); exit(1); }

    char msg[2048];
    socklen_t len, flen;
    struct sockaddr_in from;
    flen = sizeof(struct sockaddr_in);
    printf("En attente de connexion\n");
    len = listen(socket_serv, 10);
    if (len<0) { perror("c"); exit(1); }
    while(1){
        int client_fd = accept(socket_serv, (struct sockaddr*) &from, &flen);
        if (client_fd<0) { printf("%d",client_fd);perror("d"); exit(1); }
        printf("client_fd : %d\n", client_fd);  
        len = recvfrom(client_fd, msg, sizeof(msg), 0, (struct sockaddr*) &from, &flen);

        printf("msg reçu : %s", msg);

        char * reponse = malloc(200);
        memset(reponse, 's', 200);
        char * header = "HTTP/1.1 200 OK\r\nContent-Type: ratio\r\n\r\n<!DOCTYPE html> ";
        memcpy(reponse, header, strlen(header));

        int file_fd = open("index.txt", O_RDONLY);
        int readd = read(file_fd, reponse + strlen(header), 100);
        printf("octet lu : %d | %s", readd, reponse);


        int send_controle = send(client_fd, reponse, 200, 0);
        printf("\nsend_controle : %d\n",send_controle);
        if (send_controle<0) {perror("e"); exit(1);}
        close(client_fd);
    }
}