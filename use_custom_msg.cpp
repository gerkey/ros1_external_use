#include <myproject/Foo.h>
#include <stdio.h>
int
main(void)
{
  myproject::Foo foo;
  foo.foo = 42;
  printf("%d\n", foo.foo);
}

