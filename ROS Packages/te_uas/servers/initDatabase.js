import bcrypt from 'bcrypt';
import { LowSync } from 'lowdb';
import { JSONFileSync } from 'lowdb/node';

const defaultData = { users: [] };
const databasePath = process.env.TE_UAS_AUTH_PATH; // Path to the database file
const db = new LowSync(new JSONFileSync(databasePath), defaultData);

db.read();
const saltRounds = 10;

const users = [
  {  pin: bcrypt.hashSync('0000', saltRounds), id: 12032, name: 'Llama User', role: "Administrator" },
];

db.data.users.push(...users);
db.write();
